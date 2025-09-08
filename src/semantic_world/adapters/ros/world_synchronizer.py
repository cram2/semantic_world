import json
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from functools import cached_property
from typing import ClassVar

import numpy as np
import rclpy  # type: ignore
import std_msgs.msg
from ormatic.dao import to_dao
from random_events.utils import SubclassJSONSerializer
from rclpy.node import Node as RosNode
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sqlalchemy import select
from sqlalchemy.orm import Session
from typing_extensions import Callable

from .messages import MetaData, WorldStateUpdate, Message, ModificationBlock, LoadModel
from ...orm.ormatic_interface import *
from ...world import World
from ...world_description.world_modification import (
    WorldModelModificationBlock,
)


@dataclass
class Synchronizer(ABC):
    """
    Abstract Synchronizer class to manage world synchronizations between processes running semantic world.
    It manages publishers and subscribers, ensuring proper cleanup after use.
    The communication is JSON string based.
    """

    node: RosNode
    """
    The rclpy node used to create the publishers and subscribers.
    """

    world: World
    """
    The world to synchronize.
    """

    topic_name: Optional[str] = None
    """
    The topic name of the publisher and subscriber.
    """

    publisher: Optional[Publisher] = field(init=False, default=None)
    """
    The publisher used to publish the world state.
    """

    subscriber: Optional[Subscription] = field(default=None, init=False)
    """
    The subscriber to the world state.
    """

    message_type: ClassVar[Optional[Type[SubclassJSONSerializer]]] = None

    def __post_init__(self):
        self.publisher = self.node.create_publisher(
            std_msgs.msg.String, topic=self.topic_name, qos_profile=10
        )
        self.subscriber = self.node.create_subscription(
            std_msgs.msg.String,
            topic=self.topic_name,
            callback=self.subscription_callback,
            qos_profile=10,
        )

    @cached_property
    def meta_data(self) -> MetaData:
        """
        The metadata of the synchronizer which can be used to compare origins of messages.
        """
        return MetaData(
            node_name=self.node.get_name(), process_id=os.getpid(), object_id=id(self)
        )

    @abstractmethod
    def subscription_callback(self, msg):
        raise NotImplementedError

    def publish(self, msg: Message):
        self.publisher.publish(std_msgs.msg.String(data=json.dumps(msg.to_json())))

    def close(self):
        """
        Clean up publishers and subscribers.
        """

        # Destroy subscribers
        if self.subscriber is not None:
            self.node.destroy_subscription(self.subscriber)
            self.subscriber = None

        # Destroy publishers
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None


@dataclass
class SynchronizerOnCallback(Synchronizer, ABC):
    """
    Synchronizer that does something on callbacks by the world.
    Additionally, ensures that the callback is cleaned up on close.
    """

    _callback: Optional[Callable] = field(init=False, default=None)
    """
    The callback function called by the world.
    """

    _skip_next_world_callback: bool = False
    """
    Flag to indicate if the next world callback should be skipped.
    
    An incoming message from some other world might trigger a change in this world that produces a notify callback that 
    will try to send a message. 
    If the callback is triggered by a message, this synchronizer should not republish the change.
    """

    def __post_init__(self):
        super().__post_init__()
        self._callback = lambda: self.world_callback_handler()

    def subscription_callback(self, msg: std_msgs.msg.String):
        """
        Wrap the origin subscription callback by self-skipping and disabling the next world callback.
        """
        msg = self.message_type.from_json(json.loads(msg.data))
        if msg.meta_data == self.meta_data:
            return
        self._skip_next_world_callback = True
        self._subscription_callback(msg)

    @abstractmethod
    def _subscription_callback(self, msg):
        """
        Callback function called when receiving new messages from other publishers.
        """
        raise NotImplementedError

    def world_callback_handler(self):
        """
        Wrapper method around world_callback that checks if this time the callback should be triggered.
        """
        if self._skip_next_world_callback:
            self._skip_next_world_callback = False
        else:
            self.world_callback()

    @abstractmethod
    def world_callback(self):
        """
        Called when the world notifies an update.
        """
        raise NotImplementedError

    def close(self):
        if self._callback in self.world.state_change_callbacks:
            self.world.state_change_callbacks.remove(self._callback)
        if self._callback in self.world.model_change_callbacks:
            self.world.model_change_callbacks.remove(self._callback)
        self._callback = None
        super().close()


@dataclass
class StateSynchronizer(SynchronizerOnCallback):
    """
    Synchronizes the state (values of free variables) of the semantic world with the associated ROS topic.
    """

    message_type: ClassVar[Optional[Type[SubclassJSONSerializer]]] = WorldStateUpdate

    topic_name: str = "/semantic_world/world_state"

    previous_world_state_data: np.ndarray = field(init=False, default=None)
    """
    The previous world state data used to check if something changed.
    """

    def __post_init__(self):
        super().__post_init__()
        self.update_previous_world_state()
        self.world.state_change_callbacks.append(self._callback)

    def update_previous_world_state(self):
        """
        Update the previous world state to reflect the current world positions.
        """
        self.previous_world_state_data = np.copy(self.world.state.positions)

    def _subscription_callback(self, msg: WorldStateUpdate):
        """
        Update the world state with the provided message.

        :param msg: The message containing the new state information.
        """
        # Parse incoming states: WorldState has 'states' only
        indices = [self.world.state._index[n] for n in msg.prefixed_names]

        if indices:
            self.world.state.data[0, indices] = np.asarray(msg.states, dtype=float)
            self.update_previous_world_state()
            self.world.notify_state_change()

    def world_callback(self):
        """
        Publish the current world state to the ROS topic.
        """
        changes = {
            name: current_state
            for name, previous_state, current_state in zip(
                self.world.state.keys(),
                self.previous_world_state_data,
                self.world.state.positions,
            )
            if not np.allclose(previous_state, current_state)
        }

        if not changes:
            return

        msg = WorldStateUpdate(
            prefixed_names=list(changes.keys()),
            states=list(changes.values()),
            meta_data=self.meta_data,
        )
        self.update_previous_world_state()
        self.publish(msg)


@dataclass
class ModelSynchronizer(SynchronizerOnCallback):
    """
    Synchronizes the model (addition/removal of bodies/DOFs/connections) with the associated ROS topic.
    """

    message_type: ClassVar[Type[SubclassJSONSerializer]] = ModificationBlock
    topic_name: str = "/semantic_world/world_model"

    def __post_init__(self):
        super().__post_init__()
        self.world.model_change_callbacks.append(self._callback)

    def _subscription_callback(self, msg: ModificationBlock):
        msg.modifications.apply(self.world)

    def world_callback(self):
        latest_changes = WorldModelModificationBlock.from_modifications(
            self.world._atomic_modifications[-1]
        )
        msg = ModificationBlock(
            meta_data=self.meta_data,
            modifications=latest_changes,
        )
        self.publish(msg)


@dataclass
class ModelReloadSynchronizer(Synchronizer):
    """
    Synchronizes the model reloading process across different systems using ROS messaging.
    The database must be the same across the different processes, otherwise the synchronizer will fail.

    Use this when you did changes to the model that cannot be communicated via the ModelSynchronizer and hence need
    to force all processes to load your world model. Note that this may take a couple of seconds.
    """

    message_type: ClassVar[Type[SubclassJSONSerializer]] = LoadModel

    session: Session = None
    """
    The session used to perform persistence interaction. 
    """

    topic_name: str = "/semantic_world/reload_model"

    def __post_init__(self):
        super().__post_init__()
        assert self.session is not None

    def publish_reload_model(self):
        """
        Save the current world model to the database and publish the primary key to the ROS topic such that other
        processes can subscribe to the model changes and update their worlds.
        """
        dao: WorldMappingDAO = to_dao(self.world)
        self.session.add(dao)
        self.session.commit()
        message = LoadModel(primary_key=dao.id, meta_data=self.meta_data)
        self.publish(message)

    def subscription_callback(self, msg: std_msgs.msg.String):
        """
        Update the world with the new model by fetching it from the database.

        :param msg: The message containing the primary key of the model to be fetched.
        """

        msg = LoadModel.from_json(json.loads(msg.data))

        if msg.meta_data == self.meta_data:
            return

        query = select(WorldMappingDAO).where(WorldMappingDAO.id == msg.primary_key)
        new_world = self.session.scalars(query).one().from_dao()
        self._replace_world(new_world)
        self.world._notify_model_change()

    def _replace_world(self, new_world: World):
        """
        Replaces the current world with a new one, updating all relevant attributes.
        This method modifies the existing world state, kinematic structure, degrees
        of freedom, and views based on the `new_world` provided.

        If you encounter any issues with references to dead objects, it is most likely due to this method not doing
        everything needed.

        :param new_world: The new world instance to replace the current world.
        """
        with self.world.modify_world():
            self.world.clear()
            self.world.merge_world(new_world)
