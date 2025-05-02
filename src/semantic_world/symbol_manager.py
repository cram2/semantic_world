from typing import Dict, Callable, Type, Optional, overload, Union, Iterable, Tuple

import numpy as np

import semantic_world.spatial_types as cas

Provider = Union[float, Callable[[], float]]

from threading import Lock


class SingletonMeta(type):
    """
    This is a thread-safe implementation of Singleton.
    Source: https://refactoring.guru/design-patterns/singleton/python/example#example-1
    """

    _instances = {}

    _lock: Lock = Lock()

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__.py` argument do not affect
        the returned instance.
        """
        with cls._lock:
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


class SymbolManager(metaclass=SingletonMeta):
    """
    Singleton because I don't want multiple symbols with the same name to refer to different objects.
    TODO usually called registry
    Manages the association of symbolic variables with their providers and facilitates operations on them.

    The `SymbolManager` class is a tool for managing symbolic variables and their associated value providers.
    It allows the registration of various mathematical entities such as points, vectors, quaternions, and
    provides methods for resolving these symbols to their numeric values. The class also supports the
    evaluation of expressions involving these symbols.

    The purpose of this class is to abstract the management of symbolic variables and enable their seamless
    use in mathematical and symbolic computations.
    """
    symbol_to_lambda: Dict[cas.Symbol, Callable[[], float]]
    """
    A dictionary mapping symbolic variables (`cas.Symbol`) to callable functions that provide numeric values for those symbols.
    """

    def __init__(self):
        self.symbol_str_to_provider = {}

    def register_symbol(self, symbol: cas.Symbol, provider: Provider):
        if symbol in self.symbol_str_to_provider:
            raise ValueError(f"Symbol {symbol} already exists")
        self.symbol_str_to_provider[symbol] = provider

    def register_point3(self, name: str, provider: Callable[[], np.ndarray]) -> cas.Point3:
        sx, sy, sz = cas.Symbol(f'{name}.x'), cas.Symbol(f'{name}.y'), cas.Symbol(f'{name}.z')
        p = cas.Point3([sx, sy, sz])
        self.register_symbol(sx, lambda: provider()[0])
        self.register_symbol(sy, lambda: provider()[1])
        self.register_symbol(sz, lambda: provider()[2])
        return p

    def register_vector3(self, name: str, provider: Callable[[], np.ndarray]) -> cas.Vector3:
        sx, sy, sz = cas.Symbol(f'{name}.x'), cas.Symbol(f'{name}.y'), cas.Symbol(f'{name}.z')
        v = cas.Vector3([sx, sy, sz])
        self.register_symbol(sx, lambda: provider()[0])
        self.register_symbol(sy, lambda: provider()[1])
        self.register_symbol(sz, lambda: provider()[2])
        return v

    def register_quaternion(self, name: str, provider: Callable[[], Tuple[float, float, float, float]]) \
            -> cas.Quaternion:
        sw, sx, sy, sz = cas.Symbol(f'{name}.w'), cas.Symbol(f'{name}.x'), cas.Symbol(f'{name}.y'), cas.Symbol(
            f'{name}.z')
        q = cas.Quaternion((sx, sy, sz, sw))
        self.register_symbol(sx, lambda: provider()[0])
        self.register_symbol(sy, lambda: provider()[1])
        self.register_symbol(sz, lambda: provider()[2])
        self.register_symbol(sw, lambda: provider()[3])
        return q

    def resolve_symbols(self, symbols):
        try:
            return np.array([self.symbol_str_to_provider[s]() for s in symbols], dtype=float)
        except Exception as e:
            for s in symbols:
                try:
                    np.array([self.symbol_str_to_provider[s]()])
                except Exception as e2:
                    raise KeyError(f'Cannot resolve {s} ({e2.__class__.__name__}: {str(e2)})')
            raise e

    def resolve_expr(self, expr: cas.CompiledFunction):
        symbols = expr.params
        return expr.fast_call(self.resolve_symbols(symbols))

    def evaluate_expr(self, expr: cas.Expression):
        if isinstance(expr, (int, float)):
            return expr
        f = expr.compile()
        if len(f.str_params) == 0:
            return expr.to_np()
        result = f.fast_call(self.resolve_symbols(f.str_params))
        if len(result) == 1:
            return result[0]
        else:
            return result


symbol_manager = SymbolManager()
