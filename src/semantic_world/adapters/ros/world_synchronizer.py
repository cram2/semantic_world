import json
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from functools import cached_property
from typing_extensions import Callable

import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node as RosNode
import semantic_world_msgs.msg
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from ormatic.dao import to_dao
from sqlalchemy import select
from sqlalchemy.orm import Session

from ...orm.ormatic_interface import *
from ...datastructures import PrefixedName
from ...world import World
from ...world_description import (WorldModelModificationBlock, WorldModelModification, )


@dataclass
class Synchronizer(ABC):
    """
    Synchronizer class to manage world synchronizations between processes running semantic world.
    It manages publishers and subscribers, ensuring proper cleanup after use.
    """

    node: RosNode
    """
    The rclpy node used to create the publishers and subscribers.
    """

    world: World
    """
    The world to synchronize.
    """

    topic_name: str = None
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

    @cached_property
    def meta_data(self) -> semantic_world_msgs.msg.MetaData:
        """
        The metadata of the synchronizer which can be used to compare origins of messages.
        """
        return semantic_world_msgs.msg.MetaData(
            node_name=self.node.get_name(), process_id=os.getpid(), object_id=id(self)
        )

    @abstractmethod
    def subscription_callback(self, msg):
        raise NotImplementedError

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
        self._callback = lambda: self.world_callback_handler()

    def subscription_callback(self, msg):
        """
        Wrap the origin subscription callback by self-skipping and disabling the next world callback.
        """
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

    topic_name: str = "/semantic_world/world_state"

    previous_world_state_data: np.ndarray = field(init=False, default=None)
    """
    The previous world state data used to check if something changed.
    """

    def __post_init__(self):
        super().__post_init__()
        self.update_previous_world_state()
        self.publisher = self.node.create_publisher(
            semantic_world_msgs.msg.WorldState, topic=self.topic_name, qos_profile=10
        )
        self.subscriber = self.node.create_subscription(
            semantic_world_msgs.msg.WorldState,
            topic=self.topic_name,
            callback=self.subscription_callback,
            qos_profile=10,
        )
        self.world.state_change_callbacks.append(self._callback)

    def update_previous_world_state(self):
        """
        Update the previous world state to reflect the current world positions.
        """
        self.previous_world_state_data = np.copy(self.world.state.positions)

    def _subscription_callback(self, msg: semantic_world_msgs.msg.WorldState):
        """
        Update the world state with the provided message.

        :param msg: The message containing the new state information.
        """
        # Parse incoming states: WorldState has 'states' only
        indices = [
            self.world.state._index[
                PrefixedName(dof_state.name.name, dof_state.name.prefix)
            ]
            for dof_state in msg.states
        ]
        positions = [dof_state.position for dof_state in msg.states]

        if indices:
            self.world.state.data[0, indices] = np.asarray(positions, dtype=float)
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

        msg = semantic_world_msgs.msg.WorldState(
            version=self.world._state_version,
            states=[
                semantic_world_msgs.msg.DegreeOfFreedomState(
                    name=semantic_world_msgs.msg.PrefixedName(
                        name=key.name, prefix=key.prefix
                    ),
                    position=value,
                )
                for key, value in changes.items()
            ],
            meta_data=self.meta_data,
        )
        self.update_previous_world_state()
        self.publisher.publish(msg)


@dataclass
class ModelSynchronizer(SynchronizerOnCallback):
    """
    Synchronizes the model (addition/removal of bodies/DOFs/connections) with the associated ROS topic.
    """

    topic_name: str = "/semantic_world/world_model"

    def __post_init__(self):
        super().__post_init__()
        self.publisher = self.node.create_publisher(
            semantic_world_msgs.msg.WorldModelModificationBlock,
            topic=self.topic_name,
            qos_profile=10,
        )
        self.subscriber = self.node.create_subscription(
            semantic_world_msgs.msg.WorldModelModificationBlock,
            topic=self.topic_name,
            callback=self.subscription_callback,
            qos_profile=10,
        )
        self.world.model_change_callbacks.append(self._callback)

    def _subscription_callback(
        self, msg: semantic_world_msgs.msg.WorldModelModificationBlock
    ):
        changes = WorldModelModificationBlock(
            modifications=[
                WorldModelModification.from_json(json.loads(m))
                for m in msg.modifications
            ]
        )
        changes.apply(self.world)

    def world_callback(self):
        latest_changes = WorldModelModificationBlock.from_modifications(
            self.world._atomic_modifications[-1]
        )
        msg = semantic_world_msgs.msg.WorldModelModificationBlock(
            version=self.world._model_version,
            modifications=[
                json.dumps(m.to_json()) for m in latest_changes.modifications
            ],
            meta_data=self.meta_data,
        )
        self.publisher.publish(msg)


@dataclass
class ModelReloadSynchronizer(Synchronizer):
    """
    Synchronizes the model reloading process across different systems using ROS messaging.
    The database must be the same across the different processes, otherwise the synchronizer will fail.

    Use this when you did changes to the model that cannot be communicated via the ModelSynchronizer and hence need
    to force all processes to load your world model. Note that this may take a couple of seconds.
    """

    session: Session = None
    """
    The session used to perform persistence interaction. 
    """

    topic_name: str = "/semantic_world/reload_model"

    def __post_init__(self):
        assert self.session is not None
        self.publisher = self.node.create_publisher(
            semantic_world_msgs.msg.WorldModelReload,
            topic=self.topic_name,
            qos_profile=10,
        )

        self.subscriber = self.node.create_subscription(
            semantic_world_msgs.msg.WorldModelReload,
            topic=self.topic_name,
            callback=self.subscription_callback,
            qos_profile=10,
        )

    def publish_reload_model(self):
        """
        Save the current world model to the database and publish the primary key to the ROS topic such that other
        processes can subscribe to the model changes and update their worlds.
        """
        dao: WorldMappingDAO = to_dao(self.world)
        self.session.add(dao)
        self.session.commit()
        message = semantic_world_msgs.msg.WorldModelReload(
            primary_key=dao.id, meta_data=self.meta_data
        )
        self.publisher.publish(message)

    def subscription_callback(self, msg: semantic_world_msgs.msg.WorldModelReload):
        """
        Update the world with the new model by fetching it from the database.

        :param msg: The message containing the primary key of the model to be fetched.
        """

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
