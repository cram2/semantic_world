import unittest

import numpy as np
import pytest

import semantic_world.spatial_types as cas
from semantic_world.symbol_manager import symbol_manager


class TestSymbolManager(unittest.TestCase):
    def test_register_symbol(self):
        s = cas.Symbol('muh')
        symbol_manager.register_symbol(s, lambda: 0)
        assert symbol_manager.resolve_symbols([s]) == np.array([0])

    def test_register_symbol_twice(self):
        s = cas.Symbol('muh')
        symbol_manager.register_symbol(s, lambda: 0)
        with pytest.raises(ValueError):
            symbol_manager.register_symbol(s, lambda: 1)
        assert symbol_manager.resolve_symbols([s]) == np.array([0])

    def test_unknown_symbol(self):
        with pytest.raises(KeyError):
            symbol_manager.resolve_symbols([cas.Symbol('muh')])

    def test_to_expr_point3(self):
        array = np.array([1, 2, 3])

        expr = symbol_manager.register_point3('muh', lambda: array)
        assert isinstance(expr, cas.Point3)
        expr_f = expr.compile()
        assert np.all(symbol_manager.resolve_expr(expr_f) == np.array([1, 2, 3, 1]))

        array[0] = 23
        assert np.all(symbol_manager.resolve_expr(expr_f) == np.array([23, 2, 3, 1]))

    def test_expr_vector3(self):
        array = np.array([1, 2, 3])

        expr = symbol_manager.register_vector3('muh', lambda: array)
        assert isinstance(expr, cas.Vector3)
        expr_f = expr.compile()
        assert np.all(symbol_manager.resolve_expr(expr_f) == np.array([1, 2, 3, 0]))

    def test_to_expr_quaternion(self):
        array = np.array([0, 0, 0, 1])

        expr = symbol_manager.register_quaternion('muh', lambda: array)
        assert isinstance(expr, cas.Quaternion)
        expr_f = expr.compile()
        assert np.all(symbol_manager.resolve_expr(expr_f) == np.array([0, 0, 0, 1]))
        R = expr.to_rotation_matrix()
        R_f = R.compile()
        assert np.all(symbol_manager.resolve_expr(R_f) == np.eye(4))
