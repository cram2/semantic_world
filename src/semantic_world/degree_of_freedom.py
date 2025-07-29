from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import Optional, Union

from .spatial_types import spatial_types as cas
from .spatial_types.derivatives import Derivatives, DerivativeMap
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

    lower_limits: DerivativeMap[float] = field(default_factory=DerivativeMap)
    upper_limits: DerivativeMap[float] = field(default_factory=DerivativeMap)
    """
    Lower and upper bounds for each derivative
    """

    symbols: DerivativeMap[cas.Symbol] = field(default_factory=DerivativeMap, init=False)
    """
    Symbolic representations for each derivative
    """

    def __post_init__(self):
        self.lower_limits = self.lower_limits or DerivativeMap()
        self.upper_limits = self.upper_limits or DerivativeMap()
        for derivative in Derivatives.range(Derivatives.position, Derivatives.jerk):
            s = symbol_manager.register_symbol_provider(f'{self.name}_{derivative}',
                                                        lambda d=derivative: self._world.state[self.name][d])
            self.symbols.data[derivative] = s

    def reset_cache(self):
        for method_name in dir(self):
            try:
                getattr(self, method_name).memo.clear()
            except:
                pass

    @lru_cache(maxsize=None)
    def has_position_limits(self) -> bool:
        try:
            lower_limit = self.lower_limits.position
            upper_limit = self.upper_limits.position
            return lower_limit is not None or upper_limit is not None
        except KeyError:
            return False

    def __hash__(self):
        return hash(id(self))
