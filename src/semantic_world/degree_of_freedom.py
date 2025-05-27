from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Dict, Optional, Union

from .prefixed_name import PrefixedName
from .spatial_types import spatial_types as cas
from .spatial_types.derivatives import Derivatives
from .spatial_types.symbol_manager import symbol_manager
from .world_entity import WorldEntity


@dataclass
class DegreeOfFreedom(WorldEntity):
    """
    A class representing a degree of freedom in a world model with associated derivatives and limits.
    
    This class manages a variable that can freely change within specified limits, tracking its position,
    velocity, acceleration, and jerk. It maintains symbolic representations for each derivative order
    and provides methods to get and set limits for these derivatives.
    """

    name: PrefixedName
    """
    The identifier for this free variable
    """

    _lower_limits: Dict[Derivatives, Optional[float]] = field(default=None)
    _upper_limits: Dict[Derivatives, Optional[float]] = field(default=None)
    """
    Lower and upper bounds for each derivative
    """

    _lower_limits_overwrite: Dict[Derivatives, Optional[float]] = field(default_factory=dict)
    _upper_limits_overwrite: Dict[Derivatives, Optional[float]] = field(default_factory=dict)
    """
    Temporary lower and upper bound overwrites
    """

    state_idx: int = field(default=None, init=False)
    """
    Index of this variable in the world state
    """

    _derivative_symbols: Dict[Derivatives, cas.Symbol] = field(default_factory=dict, init=False)
    """
    Symbolic representations for each derivative
    """

    def __post_init__(self):
        self._lower_limits = self._lower_limits or defaultdict(lambda: None)
        self._upper_limits = self._upper_limits or defaultdict(lambda: None)
        self.state_idx = len(self._world.degrees_of_freedom)

        # Register symbols for all derivatives in one loop
        for derivative in Derivatives.range(Derivatives.position, Derivatives.jerk):
            s = cas.Symbol(f'{self.name}_{derivative}')
            self._derivative_symbols[derivative] = s
            symbol_manager.register_symbol(s, lambda d=derivative: self._world.state[d, self.state_idx])

    @property
    def position_symbol(self) -> cas.Symbol:
        return self.get_symbol(Derivatives.position)

    @property
    def velocity_symbol(self) -> cas.Symbol:
        return self.get_symbol(Derivatives.velocity)

    @property
    def acceleration_symbol(self) -> cas.Symbol:
        return self.get_symbol(Derivatives.acceleration)

    @property
    def jerk_symbol(self) -> cas.Symbol:
        return self.get_symbol(Derivatives.jerk)

    def get_symbol(self, derivative: Derivatives) -> Union[cas.Symbol, float]:
        try:
            return self._derivative_symbols[derivative]
        except KeyError:
            raise KeyError(f'Free variable {self} doesn\'t have symbol for derivative of order {derivative}')

    def reset_cache(self):
        for method_name in dir(self):
            try:
                getattr(self, method_name).memo.clear()
            except:
                pass

    @lru_cache(maxsize=None)
    def get_lower_limit(self, derivative: Derivatives) -> Optional[float]:
        if derivative in self._lower_limits and derivative in self._lower_limits_overwrite:
            lower_limit = cas.max(self._lower_limits[derivative], self._lower_limits_overwrite[derivative])
        elif derivative in self._lower_limits:
            lower_limit = self._lower_limits[derivative]
        elif derivative in self._lower_limits_overwrite:
            lower_limit = self._lower_limits_overwrite[derivative]
        else:
            return None
        return lower_limit

    @lru_cache(maxsize=None)
    def get_upper_limit(self, derivative: Derivatives) -> Optional[float]:
        if derivative in self._upper_limits and derivative in self._upper_limits_overwrite:
            upper_limit = cas.min(self._upper_limits[derivative], self._upper_limits_overwrite[derivative])
        elif derivative in self._upper_limits:
            upper_limit = self._upper_limits[derivative]
        elif derivative in self._upper_limits_overwrite:
            upper_limit = self._upper_limits_overwrite[derivative]
        else:
            return None
        return upper_limit

    def set_lower_limit(self, derivative: Derivatives, limit: float):
        self._lower_limits_overwrite[derivative] = limit

    def set_upper_limit(self, derivative: Derivatives, limit: float):
        self._upper_limits_overwrite[derivative] = limit

    @lru_cache(maxsize=None)
    def has_position_limits(self) -> bool:
        try:
            lower_limit = self.get_lower_limit(Derivatives.position)
            upper_limit = self.get_upper_limit(Derivatives.position)
            return lower_limit is not None or upper_limit is not None
        except KeyError:
            return False

    def __hash__(self):
        return hash(id(self))
