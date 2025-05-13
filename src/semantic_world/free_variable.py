from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Optional, Union, TYPE_CHECKING

import semantic_world.spatial_types.spatial_types as cas
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives
from .spatial_types.symbol_manager import symbol_manager

if TYPE_CHECKING:
    from .world import World


@dataclass
class FreeVariable:
    """
    Stub class for free variables.
    """

    state_idx: int

    def __init__(self,
                 name: PrefixedName,
                 lower_limits: Dict[Derivatives, float],
                 upper_limits: Dict[Derivatives, float],
                 world: World,
                 horizon_functions: Optional[Dict[Derivatives, float]] = None):
        self._symbols = {}
        self.state_idx = len(world.free_variables) - 1
        self.name = name

        s = cas.Symbol(f'{self.name}_{Derivatives.position}')
        self._symbols[Derivatives.position] = s
        symbol_manager.register_symbol(s, lambda: world.position_state[self.state_idx])

        s = cas.Symbol(f'{self.name}_{Derivatives.velocity}')
        self._symbols[Derivatives.velocity] = s
        symbol_manager.register_symbol(s, lambda: world.velocity_state[self.state_idx])

        s = cas.Symbol(f'{self.name}_{Derivatives.acceleration}')
        self._symbols[Derivatives.acceleration] = s
        symbol_manager.register_symbol(s, lambda: world.acceleration_state[self.state_idx])

        s = cas.Symbol(f'{self.name}_{Derivatives.jerk}')
        self._symbols[Derivatives.jerk] = s
        symbol_manager.register_symbol(s, lambda: world.jerk_state[self.state_idx])

        self.position_name = str(self._symbols[Derivatives.position])
        self.default_lower_limits = lower_limits
        self.default_upper_limits = upper_limits
        self.lower_limits = {}
        self.upper_limits = {}
        assert max(self._symbols.keys()) == len(self._symbols) - 1

        self.horizon_functions = defaultdict(lambda: 0.00001)
        if horizon_functions is None:
            horizon_functions = {Derivatives.velocity: 1,
                                 Derivatives.acceleration: 0.1,
                                 Derivatives.jerk: 0.1}
        self.horizon_functions.update(horizon_functions)

    def get_symbol(self, derivative: Derivatives) -> Union[cas.Symbol, float]:
        try:
            return self._symbols[derivative]
        except KeyError:
            raise KeyError(f'Free variable {self} doesn\'t have symbol for derivative of order {derivative}')

    def reset_cache(self):
        for method_name in dir(self):
            try:
                getattr(self, method_name).memo.clear()
            except:
                pass

    # @memoize
    def get_lower_limit(self, derivative: Derivatives, default: bool = False, evaluated: bool = False) \
            -> Union[cas.Expression, float]:
        if not default and derivative in self.default_lower_limits and derivative in self.lower_limits:
            expr = cas.max(self.default_lower_limits[derivative], self.lower_limits[derivative])
        elif derivative in self.default_lower_limits:
            expr = self.default_lower_limits[derivative]
        elif derivative in self.lower_limits:
            expr = self.lower_limits[derivative]
        else:
            raise KeyError(f'Free variable {self} doesn\'t have lower limit for derivative of order {derivative}')
        if evaluated:
            if expr is None:
                return None
            return float(symbol_manager.evaluate_expr(expr))
        return expr

    def set_lower_limit(self, derivative: Derivatives, limit: Union[cas.Expression, float]):
        self.lower_limits[derivative] = limit

    def set_upper_limit(self, derivative: Derivatives, limit: Union[Union[cas.Symbol, float], float]):
        self.upper_limits[derivative] = limit

    # @memoize
    def get_upper_limit(self, derivative: Derivatives, default: bool = False, evaluated: bool = False) \
            -> Union[Union[cas.Symbol, float], float]:
        if not default and derivative in self.default_upper_limits and derivative in self.upper_limits:
            expr = cas.min(self.default_upper_limits[derivative], self.upper_limits[derivative])
        elif derivative in self.default_upper_limits:
            expr = self.default_upper_limits[derivative]
        elif derivative in self.upper_limits:
            expr = self.upper_limits[derivative]
        else:
            raise KeyError(f'Free variable {self} doesn\'t have upper limit for derivative of order {derivative}')
        if evaluated:
            if expr is None:
                return None
            return symbol_manager.evaluate_expr(expr)
        return expr

    def get_lower_limits(self, max_derivative: Derivatives) -> Dict[Derivatives, float]:
        lower_limits = {}
        for derivative in Derivatives.range(Derivatives.position, max_derivative):
            lower_limits[derivative] = self.get_lower_limit(derivative, default=False, evaluated=True)
        return lower_limits

    def get_upper_limits(self, max_derivative: Derivatives) -> Dict[Derivatives, float]:
        upper_limits = {}
        for derivative in Derivatives.range(Derivatives.position, max_derivative):
            upper_limits[derivative] = self.get_upper_limit(derivative, default=False, evaluated=True)
        return upper_limits

    # @memoize
    def has_position_limits(self) -> bool:
        try:
            lower_limit = self.get_lower_limit(Derivatives.position)
            upper_limit = self.get_upper_limit(Derivatives.position)
            return lower_limit is not None and upper_limit is not None
        except KeyError:
            return False

    # @memoize
    def normalized_weight(self, t: int, derivative: Derivatives, prediction_horizon: int, alpha: float,
                          evaluated: bool = False) -> Union[Union[cas.Symbol, float], float]:
        limit = self.get_upper_limit(derivative)
        if limit is None:
            return 0.0
        weight = symbol_manager.evaluate_expr(self.quadratic_weights[derivative])
        limit = symbol_manager.evaluate_expr(limit)

        start = weight * alpha
        a = (weight - start) / (prediction_horizon)
        weight_scaled = a * t + start

        return weight_scaled * (1 / limit) ** 2

    def __str__(self) -> str:
        return self.position_name

    def __repr__(self):
        return str(self)
