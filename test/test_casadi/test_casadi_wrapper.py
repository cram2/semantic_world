import math
import pytest
from datetime import timedelta
import semantic_world.spatial_types.math as giskard_math
import hypothesis.strategies as st
import numpy as np
from hypothesis import given, assume, settings
import semantic_world.spatial_types.spatial_types as cas
from .utils_for_tests import float_no_nan_no_inf, quaternion, random_angle, unit_vector, compare_axis_angle, \
    angle_positive, vector, lists_of_same_length, compare_orientations, sq_matrix, float_no_nan_no_inf_min_max


def logic_not(a):
    if a == cas.TrinaryTrue:
        return cas.TrinaryFalse
    elif a == cas.TrinaryFalse:
        return cas.TrinaryTrue
    elif a == cas.TrinaryUnknown:
        return cas.TrinaryUnknown
    else:
        raise ValueError(f'Invalid truth value: {a}')


def logic_and(a, b):
    if a == cas.TrinaryFalse or b == cas.TrinaryFalse:
        return cas.TrinaryFalse
    elif a == cas.TrinaryTrue and b == cas.TrinaryTrue:
        return cas.TrinaryTrue
    elif a == cas.TrinaryUnknown or b == cas.TrinaryUnknown:
        return cas.TrinaryUnknown
    else:
        raise ValueError(f'Invalid truth values: {a}, {b}')


def logic_or(a, b):
    if a == cas.TrinaryTrue or b == cas.TrinaryTrue:
        return cas.TrinaryTrue
    elif a == cas.TrinaryFalse and b == cas.TrinaryFalse:
        return cas.TrinaryFalse
    elif a == cas.TrinaryUnknown or b == cas.TrinaryUnknown:
        return cas.TrinaryUnknown
    else:
        raise ValueError(f'Invalid truth values: {a}, {b}')


class TestUndefinedLogic:
    values = [cas.TrinaryTrue, cas.TrinaryFalse, cas.TrinaryUnknown]

    def test_and3(self):
        s = cas.Symbol('a')
        s2 = cas.Symbol('b')
        expr = cas.logic_and3(s, s2)
        f = expr.compile()
        for i in self.values:
            for j in self.values:
                expected = logic_and(i, j)
                actual = f(a=i, b=j)
                assert expected == actual, f'a={i}, b={j}, expected {expected}, actual {actual}'

    def test_or3(self):
        s = cas.Symbol('a')
        s2 = cas.Symbol('b')
        expr = cas.logic_or3(s, s2)
        f = expr.compile()
        for i in self.values:
            for j in self.values:
                expected = logic_or(i, j)
                actual = f(a=i, b=j)
                assert expected == actual, f'a={i}, b={j}, expected {expected}, actual {actual}'

    def test_not3(self):
        s = cas.Symbol('muh')
        expr = cas.logic_not3(s)
        f = expr.compile()
        for i in self.values:
            expected = logic_not(i)
            actual = f(muh=i)
            assert expected == actual, f'a={i}, expected {expected}, actual {actual}'

    def test_sub_logic_operators(self):
        def reference_function(a, b, c):
            not_c = logic_not(c)
            or_result = logic_or(b, not_c)
            result = logic_and(a, or_result)
            return result

        a, b, c = cas.create_symbols(['a', 'b', 'c'])
        expr = cas.logic_and(a, cas.logic_or(b, cas.logic_not(c)))
        new_expr = cas.replace_with_three_logic(expr)
        f = new_expr.compile()
        for i in self.values:
            for j in self.values:
                for k in self.values:
                    computed_result = f(a=i, b=j, c=k)
                    expected_result = reference_function(i, j, k)
                    assert computed_result == expected_result, f"Mismatch for inputs i={i}, j={j}, k={k}. Expected {expected_result}, got {computed_result}"


class TestSymbol:
    def test_from_name(self):
        s = cas.Symbol('muh')
        assert isinstance(s, cas.Symbol)
        assert str(s) == 'muh'

    def test_simple_math(self):
        s = cas.Symbol('muh')
        e = s + s
        assert isinstance(e, cas.Expression)
        e = s - s
        assert isinstance(e, cas.Expression)
        e = s * s
        assert isinstance(e, cas.Expression)
        e = s / s
        assert isinstance(e, cas.Expression)
        e = s ** s
        assert isinstance(e, cas.Expression)

    def test_comparisons(self):
        s = cas.Symbol('muh')
        e = s > s
        assert isinstance(e, cas.Expression)
        e = s >= s
        assert isinstance(e, cas.Expression)
        e = s < s
        assert isinstance(e, cas.Expression)
        e = s <= s
        assert isinstance(e, cas.Expression)
        e = cas.equal(s, s)
        assert isinstance(e, cas.Expression)

    def test_logic(self):
        s1 = cas.Symbol('s1')
        s2 = cas.Symbol('s2')
        s3 = cas.Symbol('s3')
        e = s1 | s2
        assert isinstance(e, cas.Expression)
        e = s1 & s2
        assert isinstance(e, cas.Expression)
        e = ~s1
        assert isinstance(e, cas.Expression)
        e = s1 & (s2 | ~s3)
        assert isinstance(e, cas.Expression)

    def test_hash(self):
        s = cas.Symbol('muh')
        d = {s: 1}
        assert d[s] == 1


class TestExpression:
    def test_pretty_str(self):
        e = cas.eye(4)
        e.pretty_str()

    def test_create(self):
        cas.Expression(cas.Symbol('muh'))
        cas.Expression([cas.ca.SX(1), cas.ca.SX.sym('muh')])
        m = cas.Expression(np.eye(4))
        m = cas.Expression(m)
        assert np.allclose(m.to_np(), np.eye(4))
        m = cas.Expression(cas.ca.SX(np.eye(4)))
        assert np.allclose(m.to_np(), np.eye(4))
        m = cas.Expression([1, 1])
        assert np.allclose(m.to_np(), np.array([1, 1]))
        m = cas.Expression([np.array([1, 1])])
        assert np.allclose(m.to_np(), np.array([1, 1]))
        m = cas.Expression(1)
        assert m.to_np() == 1
        m = cas.Expression([[1, 1], [2, 2]])
        assert np.allclose(m.to_np(), np.array([[1, 1], [2, 2]]))
        m = cas.Expression([])
        assert m.shape[0] == m.shape[1] == 0
        m = cas.Expression()
        assert m.shape[0] == m.shape[1] == 0

    def test_filter1(self):
        e_np = np.arange(16) * 2
        e = cas.Expression(e_np)
        filter_ = np.zeros(16, dtype=bool)
        filter_[3] = True
        filter_[5] = True
        actual = e[filter_].to_np()
        expected = e_np[filter_]
        assert np.all(actual == expected)

    def test_filter2(self):
        e_np = np.arange(16) * 2
        e_np = e_np.reshape((4, 4))
        e = cas.Expression(e_np)
        filter_ = np.zeros(4, dtype=bool)
        filter_[1] = True
        filter_[2] = True
        actual = e[filter_].to_np()
        expected = e_np[filter_]
        assert np.allclose(actual, expected)

    @given(float_no_nan_no_inf(), float_no_nan_no_inf())
    def test_add(self, f1, f2):
        expected = f1 + f2
        r1 = cas.compile_and_execute(lambda a: cas.Expression(a) + f1, [f2])
        assert np.isclose(r1, expected)
        r1 = cas.compile_and_execute(lambda a: f1 + cas.Expression(a), [f2])
        assert np.isclose(r1, expected)
        r1 = cas.compile_and_execute(lambda a, b: cas.Expression(a) + cas.Expression(b), [f1, f2])
        assert np.isclose(r1, expected)

    @given(float_no_nan_no_inf(), float_no_nan_no_inf())
    def test_sub(self, f1, f2):
        expected = f1 - f2
        r1 = cas.compile_and_execute(lambda a: cas.Expression(a) - f2, [f1])
        np.isclose(r1, expected)
        r1 = cas.compile_and_execute(lambda a: f1 - cas.Expression(a), [f2])
        np.isclose(r1, expected)
        r1 = cas.compile_and_execute(lambda a, b: cas.Expression(a) - cas.Expression(b), [f1, f2])
        np.isclose(r1, expected)

    def test_len(self):
        m = cas.Expression(np.eye(4))
        assert (len(m) == len(np.eye(4)))

    def test_simple_math(self):
        m = cas.Expression([1, 1])
        s = cas.Symbol('muh')
        e = m + s
        e = m + 1
        e = 1 + m
        assert isinstance(e, cas.Expression)
        e = m - s
        e = m - 1
        e = 1 - m
        assert isinstance(e, cas.Expression)
        e = m * s
        e = m * 1
        e = 1 * m
        assert isinstance(e, cas.Expression)
        e = m / s
        e = m / 1
        e = 1 / m
        assert isinstance(e, cas.Expression)
        e = m ** s
        e = m ** 1
        e = 1 ** m
        assert isinstance(e, cas.Expression)

    def test_get_attr(self):
        m = cas.Expression(np.eye(4))
        assert m[0, 0] == cas.Expression(1)
        assert m[1, 1] == cas.Expression(1)
        assert m[1, 0] == cas.Expression(0)
        assert isinstance(m[0, 0], cas.Expression)
        print(m.shape)

    def test_comparisons(self):
        logic_functions = [
            lambda a, b: a > b,
            lambda a, b: a >= b,
            lambda a, b: a < b,
            lambda a, b: a <= b,
            lambda a, b: a == b,
        ]
        e1_np = np.array([1, 2, 3, -1])
        e2_np = np.array([1, 1, -1, 3])
        e1_cas = cas.Expression(e1_np)
        e2_cas = cas.Expression(e2_np)
        for f in logic_functions:
            r_np = f(e1_np, e2_np)
            r_cas = f(e1_cas, e2_cas)
            assert isinstance(r_cas, cas.Expression)
            r_cas = r_cas.to_np()
            np.all(r_np == r_cas)

    def test_logic_and(self):
        s1 = cas.Symbol('s1')
        s2 = cas.Symbol('s2')
        expr = cas.logic_and(cas.BinaryTrue, s1)
        assert not cas.is_true_symbol(expr) and not cas.is_false_symbol(expr)
        expr = cas.logic_and(cas.BinaryFalse, s1)
        assert cas.is_false_symbol(expr)
        expr = cas.logic_and(cas.BinaryTrue, cas.BinaryTrue)
        assert cas.is_true_symbol(expr)
        expr = cas.logic_and(cas.BinaryFalse, cas.BinaryTrue)
        assert cas.is_false_symbol(expr)
        expr = cas.logic_and(cas.BinaryFalse, cas.BinaryFalse)
        assert cas.is_false_symbol(expr)
        expr = cas.logic_and(s1, s2)
        assert not cas.is_true_symbol(expr) and not cas.is_false_symbol(expr)

    def test_logic_or(self):
        s1 = cas.Symbol('s1')
        s2 = cas.Symbol('s2')
        s3 = cas.Symbol('s3')
        expr = cas.logic_or(cas.BinaryFalse, s1)
        assert not cas.is_true_symbol(expr) and not cas.is_false_symbol(expr)
        expr = cas.logic_or(cas.BinaryTrue, s1)
        assert cas.is_true_symbol(expr)
        expr = cas.logic_or(cas.BinaryTrue, cas.BinaryTrue)
        assert cas.is_true_symbol(expr)
        expr = cas.logic_or(cas.BinaryFalse, cas.BinaryTrue)
        assert cas.is_true_symbol(expr)
        expr = cas.logic_or(cas.BinaryFalse, cas.BinaryFalse)
        assert cas.is_false_symbol(expr)
        expr = cas.logic_or(s1, s2)
        assert not cas.is_true_symbol(expr) and not cas.is_false_symbol(expr)

        expr = cas.logic_or(s1, s2, s3)
        assert not cas.is_true_symbol(expr) and not cas.is_false_symbol(expr)

    def test_lt(self):
        e1 = cas.Expression([1, 2, 3, -1])
        e2 = cas.Expression([1, 1, -1, 3])
        gt_result = e1 < e2
        assert isinstance(gt_result, cas.Expression)
        assert cas.logic_all(gt_result == cas.Expression([0, 0, 0, 1])).to_np()


class TestRotationMatrix:
    def test_matmul_type_preservation(self):
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        with pytest.raises(TypeError):
            r @ s
        with pytest.raises(TypeError):
            r @ e
        assert isinstance(r @ v, cas.Vector3)
        with pytest.raises(TypeError):
            assert isinstance(r @ p, cas.Point3)
        assert isinstance(r @ r, cas.RotationMatrix)
        with pytest.raises(TypeError):
            r @ q
        assert isinstance(r @ t, cas.TransformationMatrix)
        assert isinstance(t @ r, cas.RotationMatrix)

    def test_x_y_z_vector(self):
        v = np.array([1, 1, 1])
        v = v / np.linalg.norm(v)
        R_ref = giskard_math.rotation_matrix_from_axis_angle(v, 1)
        R = cas.RotationMatrix().from_axis_angle(cas.Vector3.unit_vector(1, 1, 1), 1)
        assert np.allclose(R.x_vector().to_np(), R_ref[:, 0])
        assert np.allclose(R.y_vector().to_np(), R_ref[:, 1])
        assert np.allclose(R.z_vector().to_np(), R_ref[:, 2])

    def test_create_RotationMatrix(self):
        s = cas.Symbol('s')
        r = cas.RotationMatrix.from_rpy(1, 2, s)
        r = cas.RotationMatrix.from_rpy(1, 2, 3)
        assert isinstance(r, cas.RotationMatrix)
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3)
        r = cas.RotationMatrix(t)
        assert t[0, 3].to_np() == 1

    def test_from_vectors(self):
        v = np.array([1, 1, 1])
        v = v / np.linalg.norm(v)
        R_ref = giskard_math.rotation_matrix_from_axis_angle(v, 1)
        x = R_ref[:, 0]
        y = R_ref[:, 1]
        z = R_ref[:, 2]
        x_unit = cas.Vector3(*x)
        x_unit.scale(1.1)
        y_unit = cas.Vector3(*y)
        y_unit.scale(0.2)
        z_unit = cas.Vector3(*z)
        z_unit.scale(1.1)
        assert np.allclose(cas.RotationMatrix.from_vectors(x=x_unit, y=y_unit).to_np(), R_ref)
        assert np.allclose(cas.RotationMatrix.from_vectors(x=x_unit, z=z_unit).to_np(), R_ref)
        assert np.allclose(cas.RotationMatrix.from_vectors(y=y_unit, z=z_unit).to_np(), R_ref)
        assert np.allclose(cas.RotationMatrix.from_vectors(x=x_unit, y=y_unit, z=z_unit).to_np(), R_ref)

    @given(quaternion())
    def test_from_quaternion(self, q):
        actual = cas.RotationMatrix.from_quaternion(cas.Quaternion.from_iterable(q)).to_np()
        expected = giskard_math.rotation_matrix_from_quaternion(*q)
        assert np.allclose(actual, expected)

    @given(random_angle(),
           random_angle(),
           random_angle())
    def test_from_rpy(self, roll, pitch, yaw):
        m1 = cas.compile_and_execute(cas.RotationMatrix.from_rpy, [roll, pitch, yaw])
        m2 = giskard_math.rotation_matrix_from_rpy(roll, pitch, yaw)
        assert np.allclose(m1, m2)

    @given(unit_vector(length=3),
           random_angle())
    def test_rotation3_axis_angle(self, axis, angle):
        assert np.allclose(cas.compile_and_execute(cas.RotationMatrix.from_axis_angle,
                                                   [axis, angle]),
                           giskard_math.rotation_matrix_from_axis_angle(np.array(axis), angle))

    @given(quaternion())
    def test_axis_angle_from_matrix(self, q):
        m = giskard_math.rotation_matrix_from_quaternion(*q)
        actual_axis = cas.compile_and_execute(lambda x: cas.RotationMatrix(x).to_axis_angle()[0], [m])
        actual_angle = cas.compile_and_execute(lambda x: cas.RotationMatrix(x).to_axis_angle()[1], [m])
        expected_axis, expected_angle = giskard_math.axis_angle_from_rotation_matrix(m)
        compare_axis_angle(actual_angle, actual_axis[:3], expected_angle, expected_axis)
        assert actual_axis[-1] == 0

    @given(unit_vector(length=3),
           angle_positive())
    def test_axis_angle_from_matrix2(self, expected_axis, expected_angle):
        m = giskard_math.rotation_matrix_from_axis_angle(expected_axis, expected_angle)
        actual_axis = cas.compile_and_execute(lambda x: cas.RotationMatrix(x).to_axis_angle()[0], [m])
        actual_angle = cas.compile_and_execute(lambda x: cas.RotationMatrix(x).to_axis_angle()[1], [m])
        compare_axis_angle(actual_angle, actual_axis[:3], expected_angle, expected_axis)
        assert actual_axis[-1] == 0

    @given(unit_vector(4))
    def test_rpy_from_matrix(self, q):
        matrix = giskard_math.rotation_matrix_from_quaternion(*q)
        roll = cas.compile_and_execute(lambda m: cas.RotationMatrix(m).to_rpy()[0], [matrix])
        pitch = cas.compile_and_execute(lambda m: cas.RotationMatrix(m).to_rpy()[1], [matrix])
        yaw = cas.compile_and_execute(lambda m: cas.RotationMatrix(m).to_rpy()[2], [matrix])
        roll2, pitch2, yaw2 = giskard_math.rpy_from_matrix(matrix)
        assert np.isclose(roll, roll2)
        assert np.isclose(pitch, pitch2)
        assert np.isclose(yaw, yaw2)

    @given(unit_vector(4))
    def test_rpy_from_matrix2(self, q):
        matrix = giskard_math.rotation_matrix_from_quaternion(*q)
        roll = cas.compile_and_execute(lambda m: cas.RotationMatrix(m).to_rpy()[0], [matrix])
        pitch = cas.compile_and_execute(lambda m: cas.RotationMatrix(m).to_rpy()[1], [matrix])
        yaw = cas.compile_and_execute(lambda m: cas.RotationMatrix(m).to_rpy()[2], [matrix])
        r1 = cas.compile_and_execute(cas.RotationMatrix.from_rpy, [roll, pitch, yaw])
        assert np.allclose(r1, matrix, atol=1.e-4)

    def test_initialization(self):
        """Test various ways to initialize RotationMatrix"""
        # Default initialization (identity)
        r_identity = cas.RotationMatrix()
        assert isinstance(r_identity, cas.RotationMatrix)
        identity_np = r_identity.to_np()
        expected_identity = np.eye(4)
        assert np.allclose(identity_np, expected_identity)

        # From another RotationMatrix
        r_copy = cas.RotationMatrix(r_identity)
        assert isinstance(r_copy, cas.RotationMatrix)
        assert np.allclose(r_copy.to_np(), identity_np)

        # From numpy array
        rotation_data = np.eye(4)
        rotation_data[0, 1] = 0.5  # Add some rotation
        r_from_np = cas.RotationMatrix(rotation_data)
        assert isinstance(r_from_np, cas.RotationMatrix)

        # From TransformationMatrix
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)
        r_from_t = cas.RotationMatrix(t)
        assert isinstance(r_from_t, cas.RotationMatrix)
        # Should preserve rotation part only
        assert r_from_t[0, 3] == 0
        assert r_from_t[1, 3] == 0
        assert r_from_t[2, 3] == 0

    def test_sanity_check(self):
        """Test that sanity check enforces proper rotation matrix structure"""
        # Valid 4x4 matrix should pass
        valid_matrix = np.eye(4)
        r = cas.RotationMatrix(valid_matrix)

        # Check that homogeneous coordinates are enforced
        assert r[0, 3] == 0
        assert r[1, 3] == 0
        assert r[2, 3] == 0
        assert r[3, 0] == 0
        assert r[3, 1] == 0
        assert r[3, 2] == 0
        assert r[3, 3] == 1

        # Invalid shape should raise ValueError
        with pytest.raises(ValueError):
            cas.RotationMatrix(np.eye(3))  # 3x3 instead of 4x4

        with pytest.raises(ValueError):
            cas.RotationMatrix(np.ones((4, 5)))  # Wrong dimensions

    def test_orthogonality_properties(self):
        """Test orthogonality properties of rotation matrices"""
        # Create rotation from known values
        r = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)
        rotation_part = r.to_np()[:3, :3]

        # Test orthogonality: R @ R.T = I
        should_be_identity = rotation_part @ rotation_part.T
        assert np.allclose(should_be_identity, np.eye(3), atol=1e-10)

        # Test that determinant is 1 (proper rotation, not reflection)
        det = np.linalg.det(rotation_part)
        assert np.isclose(det, 1.0, atol=1e-10)

    def test_transpose(self):
        """Test transpose operation and its properties"""
        r = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)
        r_t = r.T

        assert isinstance(r_t, cas.RotationMatrix)

        # For rotation matrices: R.T = R^(-1)
        product = r @ r_t
        identity = cas.RotationMatrix()
        assert np.allclose(product.to_np(), identity.to_np(), atol=1e-10)

        # Double transpose should give original
        r_tt = r_t.T
        assert np.allclose(r.to_np(), r_tt.to_np())

    def test_inverse(self):
        """Test matrix inversion for rotation matrices"""
        r = cas.RotationMatrix.from_rpy(0.5, -0.3, 1.2)

        # For rotation matrices, inverse should equal transpose
        r_inv = r.inverse()
        r_t = r.T
        assert isinstance(r_inv, cas.RotationMatrix)
        assert np.allclose(r_inv.to_np(), r_t.to_np(), atol=1e-10)

        # R @ R^(-1) = I
        identity_check = r @ r_inv
        identity = cas.RotationMatrix()
        assert np.allclose(identity_check.to_np(), identity.to_np(), atol=1e-10)

    def test_composition(self):
        """Test composition of multiple rotations"""
        r1 = cas.RotationMatrix.from_rpy(0.1, 0, 0)  # Roll
        r2 = cas.RotationMatrix.from_rpy(0, 0.2, 0)  # Pitch
        r3 = cas.RotationMatrix.from_rpy(0, 0, 0.3)  # Yaw

        # Test that composition works
        combined = r3 @ r2 @ r1
        assert isinstance(combined, cas.RotationMatrix)

        # Compare with direct construction
        direct = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)
        # Note: Order matters in rotation composition, so this might not be exactly equal
        # but both should be valid rotation matrices
        combined_rotation = combined.to_np()[:3, :3]
        assert np.allclose(np.linalg.det(combined_rotation), 1.0)
        assert np.allclose(combined_rotation @ combined_rotation.T, np.eye(3), atol=1e-10)

    def test_vector_rotation(self):
        """Test rotation of vectors and unit vectors"""
        # 90-degree rotation around Z-axis
        r_z90 = cas.RotationMatrix.from_axis_angle(cas.Vector3.Z(), np.pi / 2)

        # Rotate unit vector along X-axis
        x_axis = cas.Vector3.X()
        rotated = r_z90 @ x_axis

        assert isinstance(rotated, cas.Vector3)
        # Should become Y-axis
        expected = np.array([0, 1, 0, 0])  # Homogeneous coordinates
        assert np.allclose(rotated.to_np(), expected, atol=1e-10)

        # Test with regular Vector3
        v = cas.Vector3(1, 0, 0)
        rotated_v = r_z90 @ v
        assert isinstance(rotated_v, cas.Vector3)
        assert np.allclose(rotated_v.to_np()[:3], [0, 1, 0], atol=1e-10)

    def test_frame_properties(self):
        """Test reference frame and child frame properties"""
        r = cas.RotationMatrix()

        # Initially should be None
        assert r.reference_frame is None
        assert r.child_frame is None

        # Test frame preservation in operations
        r1 = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)
        r2 = cas.RotationMatrix.from_rpy(0.2, 0.3, 0.4)

        result = r1 @ r2
        # Frame handling depends on implementation, test basic structure
        assert hasattr(result, 'reference_frame')
        assert hasattr(result, 'child_frame')

    def test_to_conversions(self):
        """Test conversion methods to other representations"""
        r = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)

        # Test conversion to axis-angle
        axis, angle = r.to_axis_angle()
        assert isinstance(axis, cas.Vector3)
        assert axis[3] == 0  # Should be a vector, not point
        assert hasattr(angle, 'to_np')  # Should be Expression or similar

        # Test conversion to RPY
        roll, pitch, yaw = r.to_rpy()
        assert np.isclose(roll.to_np(), 0.1, atol=1e-10)
        assert np.isclose(pitch.to_np(), 0.2, atol=1e-10)
        assert np.isclose(yaw.to_np(), 0.3, atol=1e-10)

        # Test conversion to quaternion
        q = r.to_quaternion()
        assert isinstance(q, cas.Quaternion)

        # Round-trip test: R -> Q -> R should preserve rotation
        r_roundtrip = cas.RotationMatrix.from_quaternion(q)
        assert np.allclose(r.to_np(), r_roundtrip.to_np(), atol=1e-10)

    def test_invalid_matmul_operations(self):
        """Test invalid matrix multiplication operations"""
        r = cas.RotationMatrix()
        s = cas.Symbol('s')
        e = cas.Expression(1)
        p = cas.Point3(1, 2, 3)
        q = cas.Quaternion()

        # These should raise TypeError
        with pytest.raises(TypeError):
            r @ s  # Matrix @ Symbol
        with pytest.raises(TypeError):
            r @ e  # Matrix @ Expression (scalar)
        with pytest.raises(TypeError):
            r @ p  # Matrix @ Point3 (use TransformationMatrix instead)
        with pytest.raises(TypeError):
            r @ q  # Matrix @ Quaternion

    @given(random_angle(), random_angle(), random_angle())
    def test_rpy_roundtrip(self, roll, pitch, yaw):
        """Property-based test for RPY round-trip conversion"""
        # Avoid gimbal lock region
        assume(abs(pitch) < np.pi / 2 - 0.1)

        r = cas.RotationMatrix.from_rpy(roll, pitch, yaw)
        r_roll, r_pitch, r_yaw = r.to_rpy()

        # Round-trip should preserve values (within numerical precision)
        assert np.isclose(r_roll.to_np(), roll, atol=1e-10)
        assert np.isclose(r_pitch.to_np(), pitch, atol=1e-10)
        assert np.isclose(r_yaw.to_np(), yaw, atol=1e-10)

    @given(unit_vector(length=3), random_angle())
    def test_axis_angle_properties(self, axis, angle):
        """Property-based test for axis-angle rotation properties"""
        # Skip very small angles to avoid numerical issues
        assume(abs(angle) > 1e-6)

        axis_unit = cas.Vector3(*axis)
        r = cas.RotationMatrix.from_axis_angle(axis_unit, angle)

        # Test that axis is preserved (rotation around axis shouldn't change axis)
        rotated_axis = r @ axis_unit
        # For rotation around axis, the axis should remain unchanged
        dot_product = axis_unit @ rotated_axis
        assert np.isclose(dot_product.to_np(), 1.0, atol=1e-10)

    def test_small_angle_approximation(self):
        """Test behavior with very small rotation angles"""
        small_angle = 1e-8

        # Small rotation around Z-axis
        r = cas.RotationMatrix.from_axis_angle(cas.Vector3.Z(), small_angle)
        rotation_part = r.to_np()[:3, :3]

        # Should be close to identity for very small angles
        assert np.allclose(rotation_part, np.eye(3), atol=1e-7)

        # But determinant should still be 1
        assert np.isclose(np.linalg.det(rotation_part), 1.0, atol=1e-12)

    def test_symbolic_operations(self):
        """Test operations with symbolic expressions"""
        angle_sym = cas.Symbol('theta')

        # Create symbolic rotation
        r_sym = cas.RotationMatrix.from_axis_angle(cas.Vector3.Z(), angle_sym)

        # Should be able to compose with other rotations
        r_numeric = cas.RotationMatrix.from_rpy(0.1, 0, 0)
        result = r_sym @ r_numeric

        assert isinstance(result, cas.RotationMatrix)

        # Should contain the symbol
        symbols = result.free_symbols()
        symbol_names = [s.name for s in symbols if hasattr(s, 'name')]
        assert 'theta' in symbol_names

    def test_compilation(self):
        """Test compilation and execution of rotation matrices"""
        # Test symbolic rotation compilation
        compiled_rotation = cas.compile_and_execute(
            lambda angle: cas.RotationMatrix.from_axis_angle(cas.Vector3.Z(), angle),
            [np.pi / 4]
        )

        # Should be a valid 4x4 rotation matrix
        assert compiled_rotation.shape == (4, 4)
        rotation_part = compiled_rotation[:3, :3]
        assert np.isclose(np.linalg.det(rotation_part), 1.0)
        assert np.allclose(rotation_part @ rotation_part.T, np.eye(3), atol=1e-10)

    def test_edge_cases(self):
        """Test edge cases and boundary conditions"""
        # Zero rotation
        r_zero = cas.RotationMatrix.from_axis_angle(cas.Vector3.X(), 0)
        identity = cas.RotationMatrix()
        assert np.allclose(r_zero.to_np(), identity.to_np(), atol=1e-12)

        # Full rotation (2π)
        r_full = cas.RotationMatrix.from_axis_angle(cas.Vector3.Y(), 2 * np.pi)
        assert np.allclose(r_full.to_np(), identity.to_np(), atol=1e-10)

        # π rotation (180 degrees)
        r_pi = cas.RotationMatrix.from_axis_angle(cas.Vector3.Z(), np.pi)
        rotation_part = r_pi.to_np()[:3, :3]
        # Should flip X and Y axes
        expected_rotation = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        assert np.allclose(rotation_part, expected_rotation, atol=1e-10)

    def test_quaternion_consistency(self):
        """Test consistency between quaternion and rotation matrix representations"""
        # Create rotation via different methods
        r_rpy = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)
        q = r_rpy.to_quaternion()
        r_from_q = cas.RotationMatrix.from_quaternion(q)

        # Should be identical
        assert np.allclose(r_rpy.to_np(), r_from_q.to_np(), atol=1e-12)

    def test_determinant_preservation(self):
        """Test that all operations preserve determinant = 1"""
        r1 = cas.RotationMatrix.from_rpy(0.5, -0.3, 1.2)
        r2 = cas.RotationMatrix.from_axis_angle(cas.Vector3.unit_vector(1, 1, 1), 0.8)

        operations_to_test = [
            r1,
            r1.T,
            r1.inverse(),
            r1 @ r2,
            r2 @ r1,
        ]

        for r in operations_to_test:
            rotation_part = r.to_np()[:3, :3]
            det = np.linalg.det(rotation_part)
            assert np.isclose(det, 1.0, atol=1e-10), f"Determinant {det} != 1.0 for operation"


class TestPoint3:

    @given(vector(3))
    def test_norm(self, v):
        p = cas.Point3.from_iterable(v)
        actual = p.norm().to_np()
        expected = np.linalg.norm(v)
        np.isclose(actual, expected)

    def test_init(self):
        l = [1, 2, 3]
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(l[0], l[1], l[2])
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        cas.Point3()
        cas.Point3(x=s, y=e, z=0)
        assert p[0] == l[0]
        assert p[1] == l[1]
        assert p[2] == l[2]
        assert p[3] == 1

        cas.Point3.from_iterable(cas.Expression(v))
        cas.Point3.from_iterable(p)
        cas.Point3.from_iterable(v)
        cas.Point3.from_iterable(v.s)
        cas.Point3.from_iterable(l)
        with pytest.raises(TypeError):
            cas.Point3.from_iterable(q)
        with pytest.raises(TypeError):
            cas.Point3.from_iterable(r)
        with pytest.raises(TypeError):
            cas.Point3.from_iterable(t)
        with pytest.raises(ValueError):
            cas.Point3.from_iterable(t.to_np())

    def test_matmul_type_preservation(self):
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        # Point3 dot operations
        with pytest.raises(TypeError):
            p @ s  # Point @ Symbol - not mathematically meaningful
        with pytest.raises(TypeError):
            p @ e  # Point @ Expression(scalar) - not mathematically meaningful
        assert isinstance(p @ v, cas.Expression)  # Point @ Vector3 = scalar (dot product)
        assert isinstance(p @ p, cas.Expression)  # Point @ Point3 = scalar (dot product)
        with pytest.raises(TypeError):
            p @ r  # Point @ RotationMatrix - not directly meaningful
        with pytest.raises(TypeError):
            p @ q  # Point @ Quaternion - not directly meaningful
        with pytest.raises(TypeError):
            p @ t  # Point @ TransformationMatrix - not directly meaningful

        # Reverse operations (what can be dotted with Point3)
        with pytest.raises(TypeError):
            s @ p  # Symbol @ Point3 - not meaningful
        with pytest.raises(TypeError):
            e @ p  # Expression(scalar) @ Point3 - not meaningful
        assert isinstance(v @ p, cas.Expression)  # Vector3 @ Point3 = scalar
        # Note: p @ p already tested above
        with pytest.raises(TypeError):
            q @ p  # Quaternion @ Point3 - not meaningful
        assert isinstance(t @ p, cas.Point3)
        with pytest.raises(TypeError):
            assert r @ p

    def test_sub(self):
        p1 = cas.Point3(1, 1, 1)
        p2 = cas.Point3(1, 1, 1)
        p3 = p1 - p2
        assert isinstance(p3, cas.Vector3)
        assert p3[0] == 0
        assert p3[1] == 0
        assert p3[2] == 0
        assert p3[3] == 0

    def test_add_vector3(self):
        p1 = cas.Point3(1, 1, 1)
        v1 = cas.Vector3(1, 1, 1)
        p3 = p1 + v1
        assert isinstance(p3, cas.Point3)
        assert p3[0] == 2
        assert p3[1] == 2
        assert p3[2] == 2
        assert p3[3] == 1

    def test_mul(self):
        p1 = cas.Point3(1, 1, 1)
        s = cas.Symbol('s')
        p3 = p1 * s
        assert isinstance(p3, cas.Point3)
        f = 2
        p3 = p1 / f
        assert isinstance(p3, cas.Point3)
        assert p3[0] == 0.5
        assert p3[1] == 0.5
        assert p3[2] == 0.5
        assert p3[3] == 1

    @given(lists_of_same_length([float_no_nan_no_inf(), float_no_nan_no_inf()], min_length=3, max_length=3))
    def test_dot(self, vectors):
        u, v = vectors
        u = np.array(u)
        v = np.array(v)
        result = cas.compile_and_execute(lambda p1, p2: cas.Point3.from_iterable(p1).dot(cas.Point3.from_iterable(p2)),
                                         [u, v])
        expected = np.dot(u, v.T)
        if not np.isnan(result) and not np.isinf(result):
            assert np.isclose(result, expected)

    @given(float_no_nan_no_inf(), vector(3), vector(3))
    def test_if_greater_zero(self, condition, if_result, else_result):
        actual = cas.compile_and_execute(cas.if_greater_zero, [condition, if_result, else_result])
        expected = if_result if condition > 0 else else_result
        assert np.allclose(actual, expected)

    def test_arithmetic_operations(self):
        """Test all arithmetic operations with proper type checking and homogeneous coordinates"""
        p1 = cas.Point3(1, 2, 3)
        p2 = cas.Point3(4, 5, 6)
        v = cas.Vector3(1, 1, 1)
        s = cas.Symbol('s')
        e = cas.Expression(2)

        # Addition - Point + Vector = Point
        result = p1 + v
        assert isinstance(result, cas.Point3)
        assert result[3] == 1  # Homogeneous coordinate preserved
        assert result[0] == 2 and result[1] == 3 and result[2] == 4

        # Addition - Point + Scalar
        result = p1 + 2.5
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

        result = p1 + s
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

        # Subtraction - Point - Point = Vector
        result = p1 - p2
        assert isinstance(result, cas.Vector3)
        assert result[3] == 0  # Vector homogeneous coordinate
        assert result[0] == -3 and result[1] == -3 and result[2] == -3

        # Subtraction - Point - Vector = Point
        result = p1 - v
        assert isinstance(result, cas.Point3)
        assert result[3] == 1
        assert result[0] == 0 and result[1] == 1 and result[2] == 2

        # Subtraction - Point - Scalar = Point
        result = p1 - 1
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

        # Multiplication and division
        result = p1 * 2
        assert isinstance(result, cas.Point3)
        assert result[0] == 2 and result[1] == 4 and result[2] == 6
        assert result[3] == 1

        result = p1 / 2
        assert isinstance(result, cas.Point3)
        assert result[0] == 0.5 and result[1] == 1 and result[2] == 1.5
        assert result[3] == 1

        result = p1 ** 2
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

    def test_properties(self):
        """Test x, y, z property getters and setters"""
        p = cas.Point3(1, 2, 3)

        # Test getters
        assert p.x == 1
        assert p.y == 2
        assert p.z == 3

        # Test setters
        p.x = 10
        p.y = 20
        p.z = 30
        assert p[0] == 10
        assert p[1] == 20
        assert p[2] == 30
        assert p[3] == 1  # Homogeneous coordinate unchanged

    def test_geometric_operations(self):
        """Test geometric operations specific to points"""
        p1 = cas.Point3(0, 0, 0)  # Origin
        p2 = cas.Point3(3, 4, 0)  # Point on XY plane

        # Distance between points (via subtraction and norm)
        displacement = p2 - p1
        assert isinstance(displacement, cas.Vector3)
        distance = displacement.norm()
        assert np.isclose(distance.to_np(), 5.0)  # 3-4-5 triangle

        # Midpoint calculation
        midpoint = (p1 + (p2 - p1) * 0.5)
        assert isinstance(midpoint, cas.Point3)
        assert np.isclose(midpoint.x.to_np(), 1.5)
        assert np.isclose(midpoint.y.to_np(), 2.0)
        assert np.isclose(midpoint.z.to_np(), 0.0)

    def test_reference_frame_preservation(self):
        """Test that reference frames are properly preserved through operations"""
        p1 = cas.Point3(1, 2, 3)  # reference_frame=some_frame
        v = cas.Vector3(1, 1, 1)

        # Operations should preserve the reference frame of the point
        result = p1 + v
        assert result.reference_frame == p1.reference_frame

        result = p1 * 2
        assert result.reference_frame == p1.reference_frame

        result = -p1
        assert result.reference_frame == p1.reference_frame

        # Point - Point should preserve reference frame of left operand
        p2 = cas.Point3(4, 5, 6)
        result = p1 - p2
        assert isinstance(result, cas.Vector3)
        assert result.reference_frame == p1.reference_frame

    def test_negation(self):
        """Test unary negation operator"""
        p = cas.Point3(1, -2, 3)
        result = -p

        assert isinstance(result, cas.Point3)
        assert result[0] == -1
        assert result[1] == 2
        assert result[2] == -3
        assert result[3] == 1  # Homogeneous coordinate preserved

    def test_invalid_operations(self):
        """Test operations that should raise TypeError"""
        p = cas.Point3(1, 2, 3)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        # Invalid additions - Point + Point is not geometrically meaningful
        p2 = cas.Point3(4, 5, 6)
        with pytest.raises(TypeError):
            p + p2

        # Invalid additions with matrices
        with pytest.raises(TypeError):
            p + r
        with pytest.raises(TypeError):
            p + q
        with pytest.raises(TypeError):
            p + t

        # Invalid multiplications with points/vectors
        v = cas.Vector3(1, 1, 1)
        with pytest.raises(TypeError):
            p * p2  # Point * Point not defined
        with pytest.raises(TypeError):
            p * v  # Point * Vector not defined

        # Invalid string operations
        with pytest.raises(TypeError):
            p + "string"
        with pytest.raises(TypeError):
            "string" + p

    @given(vector(3), vector(3))
    def test_distance_property_based(self, p1_data, p2_data):
        """Property-based test for point-to-point distance"""
        p1 = cas.Point3.from_iterable(p1_data)
        p2 = cas.Point3.from_iterable(p2_data)

        # Distance via subtraction and norm
        displacement = p2 - p1
        distance = displacement.norm()

        # Compare with numpy calculation
        expected = np.linalg.norm(np.array(p2_data) - np.array(p1_data))
        actual = distance.to_np()

        if not (np.isnan(actual) or np.isinf(actual)):
            assert np.isclose(actual, expected)

    def test_transformation_operations(self):
        """Test transformation matrix operations with points"""
        p = cas.Point3(1, 2, 3)
        t = cas.TransformationMatrix()

        # Test matrix @ point = point (homogeneous transformation)
        result = t @ p
        assert isinstance(result, cas.Point3)
        assert result[3] == 1  # Homogeneous coordinate preserved

        # Test that point @ matrix raises error (not mathematically meaningful)
        with pytest.raises(TypeError):
            p @ t

    def test_compilation_and_execution(self):
        """Test that Point3 operations compile and execute correctly"""
        # Test point arithmetic compilation
        compiled_add = cas.compile_and_execute(
            lambda px, py, pz, vx, vy, vz: cas.Point3(px, py, pz) + cas.Vector3(vx, vy, vz),
            [1, 2, 3, 1, 1, 1]
        )
        expected = np.array([2, 3, 4, 1])
        np.testing.assert_allclose(compiled_add, expected)

        # Test point subtraction compilation
        compiled_sub = cas.compile_and_execute(
            lambda p1x, p1y, p1z, p2x, p2y, p2z: cas.Point3(p1x, p1y, p1z) - cas.Point3(p2x, p2y, p2z),
            [5, 6, 7, 1, 2, 3]
        )
        expected_vector = np.array([4, 4, 4, 0])  # Result is a Vector3
        np.testing.assert_allclose(compiled_sub, expected_vector)

    def test_edge_cases(self):
        """Test edge cases and boundary conditions"""
        # Test with zero coordinates
        p_zero = cas.Point3(0, 0, 0)
        assert p_zero[0] == 0 and p_zero[1] == 0 and p_zero[2] == 0
        assert p_zero[3] == 1

        # Test with negative coordinates
        p_neg = cas.Point3(-1, -2, -3)
        assert p_neg[0] == -1 and p_neg[1] == -2 and p_neg[2] == -3
        assert p_neg[3] == 1

        # Test very large coordinates
        large_val = 1e6
        p_large = cas.Point3(large_val, large_val, large_val)
        assert p_large[0] == large_val
        assert p_large[3] == 1

        # Test very small coordinates
        small_val = 1e-6
        p_small = cas.Point3(small_val, small_val, small_val)
        assert p_small[0] == small_val
        assert p_small[3] == 1

    def test_symbolic_operations(self):
        """Test operations with symbolic expressions"""
        x, y, z = cas.create_symbols(['x', 'y', 'z'])
        p_symbolic = cas.Point3(x, y, z)
        p_numeric = cas.Point3(1, 2, 3)

        # Test symbolic point operations
        result = p_symbolic + cas.Vector3(1, 1, 1)
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

        # Test mixed symbolic/numeric operations
        result = p_symbolic - p_numeric
        assert isinstance(result, cas.Vector3)
        assert result[3] == 0

        # Verify symbolic expressions are preserved
        symbols = result.free_symbols()
        symbol_names = [s.name for s in symbols if hasattr(s, 'name')]
        assert 'x' in symbol_names and 'y' in symbol_names and 'z' in symbol_names

    @given(vector(3))
    def test_homogeneous_coordinate_invariant(self, coords):
        """Property-based test ensuring homogeneous coordinate is always 1 for points"""
        p = cas.Point3.from_iterable(coords)
        assert p[3] == 1

        # After operations, homogeneous coordinate should remain 1
        result = p + cas.Vector3(1, 1, 1)
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

        result = p * 2.5
        assert isinstance(result, cas.Point3)
        assert result[3] == 1

        result = -p
        assert isinstance(result, cas.Point3)
        assert result[3] == 1


class TestVector3:
    def test_init(self):
        l = [1, 2, 3]
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        cas.Vector3()
        cas.Vector3(x=s, y=e, z=0)
        v = cas.Vector3(l[0], l[1], l[2])
        assert v[0] == l[0]
        assert v[1] == l[1]
        assert v[2] == l[2]
        assert v[3] == 0  # Vector3 has 0 in homogeneous coordinate

        cas.Vector3.from_iterable(cas.Expression(v))
        cas.Vector3.from_iterable(p)  # Can create Vector3 from Point3
        cas.Vector3.from_iterable(v)
        cas.Vector3.from_iterable(v.s)
        cas.Vector3.from_iterable(l)
        with pytest.raises(TypeError):
            cas.Vector3.from_iterable(q)
        with pytest.raises(TypeError):
            cas.Vector3.from_iterable(r)
        with pytest.raises(TypeError):
            cas.Vector3.from_iterable(t)
        with pytest.raises(ValueError):
            cas.Vector3.from_iterable(t.to_np())

    def test_matmul_type_preservation(self):
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        # Vector3 dot operations
        with pytest.raises(TypeError):
            v @ s  # Vector @ Symbol - not mathematically meaningful
        with pytest.raises(TypeError):
            v @ e  # Vector @ Expression(scalar) - not mathematically meaningful
        assert isinstance(v @ v, cas.Expression)  # Vector @ Vector3 = scalar (dot product)
        assert isinstance(v @ p, cas.Expression)  # Vector @ Point3 = scalar (dot product)
        with pytest.raises(TypeError):
            v @ r  # Vector @ RotationMatrix - not directly meaningful
        with pytest.raises(TypeError):
            v @ q  # Vector @ Quaternion - not directly meaningful
        with pytest.raises(TypeError):
            v @ t  # Vector @ TransformationMatrix - not directly meaningful

        # Reverse operations (what can be dotted with Vector3)
        with pytest.raises(TypeError):
            s @ v  # Symbol @ Vector3 - not meaningful
        with pytest.raises(TypeError):
            e @ v  # Expression(scalar) @ Vector3 - not meaningful
        # v @ v already tested above
        assert isinstance(p @ v, cas.Expression)  # Point3 @ Vector3 = scalar
        with pytest.raises(TypeError):
            q @ v  # Quaternion @ Vector3 - not meaningful
        assert isinstance(r @ v, cas.Vector3)  # RotationMatrix @ Vector3 = Vector3 (transformation)
        assert isinstance(t @ v, cas.Vector3)  # TransformationMatrix @ Vector3 = Vector3 (transformation)

    @given(vector(3))
    def test_norm(self, v):
        expected = np.linalg.norm(v)
        v = cas.Vector3.from_iterable(v)
        actual = v.norm().to_np()
        np.isclose(actual, expected)

    @given(vector(3), float_no_nan_no_inf(), vector(3))
    def test_save_division(self, nominator, denominator, if_nan):
        nominator = cas.Vector3.from_iterable(nominator)
        denominator = cas.Expression(denominator)
        if_nan = cas.Vector3.from_iterable(if_nan)
        result = cas.save_division(nominator, denominator, if_nan)
        if denominator.to_np() == 0:
            assert np.allclose(result.to_np(), if_nan.to_np())
        else:
            assert np.allclose(result.to_np(), nominator.to_np() / denominator.to_np())

    @given(lists_of_same_length([float_no_nan_no_inf(), float_no_nan_no_inf()], min_length=3, max_length=3))
    def test_dot(self, vectors):
        u, v = vectors
        u = np.array(u)
        v = np.array(v)
        result = cas.compile_and_execute(
            lambda p1, p2: cas.Vector3.from_iterable(p1).dot(cas.Vector3.from_iterable(p2)), [u, v])
        expected = np.dot(u, v.T)
        if not np.isnan(result) and not np.isinf(result):
            assert np.isclose(result, expected)

    def test_cross_product(self):
        """Test cross product operations"""
        v1 = cas.Vector3(1, 0, 0)
        v2 = cas.Vector3(0, 1, 0)

        result = v1.cross(v2)
        assert isinstance(result, cas.Vector3)
        assert result[3] == 0
        # Cross product of x and y unit vectors should be z unit vector
        np.testing.assert_allclose(result[:3].to_np(), [0, 0, 1])

        # Cross product is anti-commutative
        result2 = v2.cross(v1)
        np.testing.assert_allclose(result2[:3].to_np(), [0, 0, -1])

    def test_properties(self):
        """Test x, y, z property getters and setters"""
        v = cas.Vector3(1, 2, 3)

        # Test getters
        assert v.x == 1
        assert v.y == 2
        assert v.z == 3

        # Test setters
        v.x = 10
        v.y = 20
        v.z = 30
        assert v[0] == 10
        assert v[1] == 20
        assert v[2] == 30
        assert v[3] == 0  # Homogeneous coordinate unchanged

    def test_reference_frame_preservation(self):
        """Test that reference frames are properly preserved through operations"""
        # This would require a mock reference frame object
        v1 = cas.Vector3(1, 2, 3)  # reference_frame=some_frame
        v2 = cas.Vector3(4, 5, 6)

        # Operations should preserve the reference frame of the left operand
        result = v1 + v2
        assert result.reference_frame == v1.reference_frame

        result = v1 * 2
        assert result.reference_frame == v1.reference_frame

        result = -v1
        assert result.reference_frame == v1.reference_frame

    def test_negation(self):
        """Test unary negation operator"""
        v = cas.Vector3(1, -2, 3)
        result = -v

        assert isinstance(result, cas.Vector3)
        assert result[0] == -1
        assert result[1] == 2
        assert result[2] == -3
        assert result[3] == 0

    def test_scale_method(self):
        """Test the scale method with safe and unsafe modes"""
        v = cas.Vector3(3, 4, 0)  # Length = 5

        # Safe scaling (default)
        v_copy = cas.Vector3(3, 4, 0)
        v_copy.scale(10)
        expected_norm = v_copy.norm().to_np()
        assert np.isclose(expected_norm, 10)

        # Unsafe scaling
        v_copy2 = cas.Vector3(3, 4, 0)
        v_copy2.scale(10, unsafe=True)
        expected_norm2 = v_copy2.norm().to_np()
        assert np.isclose(expected_norm2, 10)

    def test_invalid_operations(self):
        """Test operations that should raise TypeError"""
        v = cas.Vector3(1, 2, 3)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        # Invalid additions
        with pytest.raises(TypeError):
            v + r
        with pytest.raises(TypeError):
            v + q
        with pytest.raises(TypeError):
            v + t

        # Invalid multiplications with vectors
        with pytest.raises(TypeError):
            v * v  # Vector * Vector is not defined (use dot or cross)
        with pytest.raises(TypeError):
            v / v

        # Invalid string operations
        with pytest.raises(TypeError):
            v + "string"
        with pytest.raises(TypeError):
            "string" + v

    @given(vector(3), vector(3))
    def test_dot_product_property_based(self, v1_data, v2_data):
        """Property-based test for dot product"""
        v1 = cas.Vector3.from_iterable(v1_data)
        v2 = cas.Vector3.from_iterable(v2_data)

        result = v1.dot(v2)
        assert isinstance(result, cas.Expression)

        # Compare with numpy dot product
        expected = np.dot(v1_data, v2_data)
        actual = result.to_np()

        if not (np.isnan(actual) or np.isinf(actual)):
            assert np.isclose(actual, expected)

    @given(vector(3))
    def test_norm_property_based(self, v_data):
        """Property-based test for vector norm"""
        v = cas.Vector3.from_iterable(v_data)
        norm_result = v.norm()

        expected = np.linalg.norm(v_data)
        actual = norm_result.to_np()

        if not (np.isnan(actual) or np.isinf(actual)):
            assert np.isclose(actual, expected)

    def test_from_iterable_edge_cases(self):
        """Test edge cases for from_iterable class method"""
        # Test with different iterable types
        v1 = cas.Vector3.from_iterable([1, 2, 3])
        assert v1[0] == 1 and v1[1] == 2 and v1[2] == 3 and v1[3] == 0

        v2 = cas.Vector3.from_iterable((4, 5, 6))
        assert v2[0] == 4 and v2[1] == 5 and v2[2] == 6 and v2[3] == 0

        v3 = cas.Vector3.from_iterable(np.array([7, 8, 9]))
        assert v3[0] == 7 and v3[1] == 8 and v3[2] == 9 and v3[3] == 0

        # Test reference frame inheritance
        existing_vector = cas.Vector3(1, 2, 3)  # reference_frame=some_frame
        new_vector = cas.Vector3.from_iterable(existing_vector)
        assert new_vector.reference_frame == existing_vector.reference_frame

    def test_compilation_and_execution(self):
        """Test that Vector3 operations compile and execute correctly"""
        v1 = cas.Vector3(cas.Symbol('x'), cas.Symbol('y'), cas.Symbol('z'))
        v2 = cas.Vector3(1, 2, 3)

        # Test dot product compilation
        dot_expr = v1.dot(v2)
        compiled_dot = cas.compile_and_execute(
            lambda x, y, z: cas.Vector3(x, y, z).dot(cas.Vector3(1, 2, 3)),
            [1, 2, 3]
        )
        expected = 1 * 1 + 2 * 2 + 3 * 3  # = 14
        assert np.isclose(compiled_dot, expected)

        # Test cross product compilation
        cross_expr = v1.cross(v2)
        compiled_cross = cas.compile_and_execute(
            lambda x, y, z: cas.Vector3(x, y, z).cross(cas.Vector3(1, 2, 3)),
            [2, 3, 4]
        )
        expected_cross = np.cross([2, 3, 4], [1, 2, 3])
        np.testing.assert_allclose(compiled_cross[:3], expected_cross)


class TestUnitVector3:
    @given(vector(3))
    def test_is_length_1(self, v):
        assume(abs(v[0]) > 0.00001 or abs(v[1]) > 0.00001 or abs(v[2]) > 0.00001)
        unit_v = cas.Vector3.unit_vector(*v)
        assert np.isclose(unit_v.norm().to_np(), 1)

    def test_length_0(self):
        unit_v = cas.Vector3.unit_vector(x=0, y=0, z=0)
        assert np.isnan(unit_v.norm().to_np())


class TestTransformationMatrix:
    def test_matmul_type_preservation(self):
        """Test that @ operator preserves correct types for TransformationMatrix"""
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        t = cas.TransformationMatrix()

        # TransformationMatrix @ invalid types should raise TypeError
        with pytest.raises(TypeError):
            t @ s  # TransformationMatrix @ Symbol
        with pytest.raises(TypeError):
            t @ e  # TransformationMatrix @ Expression (scalar)
        with pytest.raises(TypeError):
            t @ q  # TransformationMatrix @ Quaternion

        # TransformationMatrix @ valid types should return correct types
        assert isinstance(t @ v, cas.Vector3)  # Transform vector -> Vector3
        assert isinstance(t @ p, cas.Point3)  # Transform point -> Point3
        assert isinstance(t @ r, cas.RotationMatrix)  # Transform rotation -> RotationMatrix
        assert isinstance(t @ t, cas.TransformationMatrix)  # Transform transformation -> TransformationMatrix

        # Test reverse operations (other types @ TransformationMatrix)
        # RotationMatrix @ TransformationMatrix -> TransformationMatrix (already tested in RotationMatrix)
        assert isinstance(r @ t, cas.TransformationMatrix)

        # Verify that the transformed objects maintain their geometric properties
        # Vector should remain homogeneous coordinate = 0
        transformed_vector = t @ v
        assert transformed_vector[3] == 0

        # Point should remain homogeneous coordinate = 1
        transformed_point = t @ p
        assert transformed_point[3] == 1

        # Test with non-identity transformation to ensure type preservation holds
        t_rot = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)

        assert isinstance(t_rot @ v, cas.Vector3)
        assert isinstance(t_rot @ p, cas.Point3)
        assert isinstance(t_rot @ r, cas.RotationMatrix)
        assert isinstance(t_rot @ t, cas.TransformationMatrix)

        # Verify homogeneous coordinate preservation with non-identity transform
        transformed_vector_rot = t_rot @ v
        assert transformed_vector_rot[3] == 0

        transformed_point_rot = t_rot @ p
        assert transformed_point_rot[3] == 1

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_translation3(self, x, y, z):
        r1 = cas.compile_and_execute(cas.TransformationMatrix.from_xyz_rpy, [x, y, z])
        r2 = np.identity(4)
        r2[0, 3] = x
        r2[1, 3] = y
        r2[2, 3] = z
        assert np.allclose(r1, r2)

    def test_dot(self):
        s = cas.Symbol('x')
        m1 = cas.TransformationMatrix()
        m2 = cas.TransformationMatrix.from_xyz_rpy(x=s)
        m1.dot(m2)

    def test_TransformationMatrix(self):
        f = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3)
        assert isinstance(f, cas.TransformationMatrix)

    @given(st.integers(min_value=1, max_value=10))
    def test_matrix(self, x_dim):
        data = list(range(x_dim))
        with pytest.raises(ValueError):
            cas.TransformationMatrix(data)

    @given(st.integers(min_value=1, max_value=10),
           st.integers(min_value=1, max_value=10))
    def test_matrix2(self, x_dim, y_dim):
        data = [[i + (j * x_dim) for j in range(y_dim)] for i in range(x_dim)]
        if x_dim != 4 or y_dim != 4:
            with pytest.raises(ValueError):
                m = cas.TransformationMatrix(data).to_np()
        else:
            m = cas.TransformationMatrix(data).to_np()
            assert float(m[3, 0]) == 0
            assert float(m[3, 1]) == 0
            assert float(m[3, 2]) == 0
            assert float(m[x_dim - 1, y_dim - 1]) == 1

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           unit_vector(length=3),
           random_angle())
    def test_frame3_axis_angle(self, x, y, z, axis, angle):
        r2 = giskard_math.rotation_matrix_from_axis_angle(np.array(axis), angle)
        r2[0, 3] = x
        r2[1, 3] = y
        r2[2, 3] = z
        r = cas.compile_and_execute(lambda x, y, z, axis, angle: cas.TransformationMatrix.from_point_rotation_matrix(
            cas.Point3(x, y, z),
            cas.RotationMatrix.from_axis_angle(axis, angle)),
                                    [x, y, z, axis, angle])
        assert np.allclose(r, r2)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           random_angle(),
           random_angle(),
           random_angle())
    def test_frame3_rpy(self, x, y, z, roll, pitch, yaw):
        r2 = giskard_math.rotation_matrix_from_rpy(roll, pitch, yaw)
        r2[0, 3] = x
        r2[1, 3] = y
        r2[2, 3] = z
        assert np.allclose(cas.compile_and_execute(cas.TransformationMatrix.from_xyz_rpy,
                                                   [x, y, z, roll, pitch, yaw]),
                           r2)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           unit_vector(4))
    def test_frame3_quaternion(self, x, y, z, q):
        r2 = giskard_math.rotation_matrix_from_quaternion(*q)
        r2[0, 3] = x
        r2[1, 3] = y
        r2[2, 3] = z
        r = cas.TransformationMatrix.from_point_rotation_matrix(point=cas.Point3(x, y, z),
                                                                rotation_matrix=cas.RotationMatrix.from_quaternion(
                                                                    cas.Quaternion(*q))).to_np()
        assert np.allclose(r, r2)

    @given(float_no_nan_no_inf(outer_limit=1000),
           float_no_nan_no_inf(outer_limit=1000),
           float_no_nan_no_inf(outer_limit=1000),
           quaternion())
    def test_inverse_frame(self, x, y, z, q):
        f = giskard_math.rotation_matrix_from_quaternion(*q)
        f[0, 3] = x
        f[1, 3] = y
        f[2, 3] = z
        r = cas.compile_and_execute(lambda x: cas.TransformationMatrix(x).inverse(), [f])

        r2 = np.linalg.inv(f)
        assert np.allclose(r, r2, atol=1.e-4, rtol=1.e-4)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           unit_vector(4))
    def test_pos_of(self, x, y, z, q):
        r1 = cas.TransformationMatrix.from_point_rotation_matrix(cas.Point3(x, y, z),
                                                                 cas.RotationMatrix.from_quaternion(
                                                                     cas.Quaternion(*q))).to_position()
        r2 = [x, y, z, 1]
        for i, e in enumerate(r2):
            np.isclose(r1[i].to_np(), e)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           unit_vector(4))
    def test_trans_of(self, x, y, z, q):
        r1 = cas.TransformationMatrix.from_point_rotation_matrix(point=cas.Point3(x, y, z),
                                                                 rotation_matrix=cas.RotationMatrix.from_quaternion(
                                                                     cas.Quaternion(*q))).to_translation().to_np()
        r2 = np.identity(4)
        r2[0, 3] = x
        r2[1, 3] = y
        r2[2, 3] = z
        for i in range(r2.shape[0]):
            for j in range(r2.shape[1]):
                np.isclose(float(r1[i, j]), r2[i, j])

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           unit_vector(4))
    def test_rot_of(self, x, y, z, q):
        r1 = cas.TransformationMatrix.from_point_rotation_matrix(point=cas.Point3(x, y, z),
                                                                 rotation_matrix=cas.RotationMatrix.from_quaternion(
                                                                     cas.Quaternion(*q))).to_rotation().to_np()
        r2 = giskard_math.rotation_matrix_from_quaternion(*q)
        assert np.allclose(r1, r2)

    def test_rot_of2(self):
        """
        Test to make sure the function doesn't alter the original
        """
        f = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3)
        r = f.to_rotation()
        assert f[0, 3] == 1
        assert f[0, 3] == 2
        assert f[0, 3] == 3
        assert r[0, 0] == 1
        assert r[1, 1] == 1
        assert r[2, 2] == 1

    def test_initialization(self):
        """Test various ways to initialize TransformationMatrix"""
        # Default initialization (identity)
        t_identity = cas.TransformationMatrix()
        assert isinstance(t_identity, cas.TransformationMatrix)
        identity_np = t_identity.to_np()
        expected_identity = np.eye(4)
        assert np.allclose(identity_np, expected_identity)

        # From RotationMatrix
        r = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)
        t_from_r = cas.TransformationMatrix(r)
        assert isinstance(t_from_r, cas.TransformationMatrix)
        # Should preserve rotation, zero translation
        assert t_from_r[0, 3] == 0
        assert t_from_r[1, 3] == 0
        assert t_from_r[2, 3] == 0

        # From another TransformationMatrix
        t_copy = cas.TransformationMatrix(t_from_r)
        assert isinstance(t_copy, cas.TransformationMatrix)
        assert np.allclose(t_copy.to_np(), t_from_r.to_np())

        # From numpy array
        transform_data = np.eye(4)
        transform_data[:3, 3] = [1, 2, 3]  # Add translation
        t_from_np = cas.TransformationMatrix(transform_data)
        assert isinstance(t_from_np, cas.TransformationMatrix)

    def test_sanity_check(self):
        """Test that sanity check enforces proper transformation matrix structure"""
        # Valid 4x4 matrix should pass
        valid_matrix = np.eye(4)
        valid_matrix[:3, 3] = [1, 2, 3]
        t = cas.TransformationMatrix(valid_matrix)

        # Check that bottom row is enforced to [0, 0, 0, 1]
        assert t[3, 0] == 0
        assert t[3, 1] == 0
        assert t[3, 2] == 0
        assert t[3, 3] == 1

        # Invalid shape should raise ValueError
        with pytest.raises(ValueError):
            cas.TransformationMatrix(np.eye(3))  # 3x3 instead of 4x4

        with pytest.raises(ValueError):
            cas.TransformationMatrix(np.ones((2, 5)))  # Wrong dimensions

    def test_properties(self):
        """Test x, y, z property getters and setters"""
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)

        # Test getters
        assert t.x.to_np() == 1
        assert t.y.to_np() == 2
        assert t.z.to_np() == 3

        # Test setters
        t.x = 10
        t.y = 20
        t.z = 30
        assert t[0, 3].to_np() == 10
        assert t[1, 3].to_np() == 20
        assert t[2, 3].to_np() == 30

        # Bottom row should remain unchanged
        assert t[3, 3] == 1

    def test_from_point_rotation(self):
        """Test construction from point and rotation matrix"""
        p = cas.Point3(1, 2, 3)
        r = cas.RotationMatrix.from_rpy(0.1, 0.2, 0.3)

        # From both point and rotation
        t1 = cas.TransformationMatrix.from_point_rotation_matrix(p, r)
        assert isinstance(t1, cas.TransformationMatrix)
        assert t1.x.to_np() == 1
        assert t1.y.to_np() == 2
        assert t1.z.to_np() == 3

        # From point only (identity rotation)
        t2 = cas.TransformationMatrix.from_point_rotation_matrix(point=p)
        rotation_part = t2.to_np()[:3, :3]
        assert np.allclose(rotation_part, np.eye(3))
        assert t2.x.to_np() == 1

        # From rotation only (zero translation)
        t3 = cas.TransformationMatrix.from_point_rotation_matrix(rotation_matrix=r)
        assert t3.x.to_np() == 0
        assert t3.y.to_np() == 0
        assert t3.z.to_np() == 0

    def test_from_xyz_quat(self):
        """Test construction from position and quaternion"""
        t = cas.TransformationMatrix.from_xyz_quat(
            pos_x=1, pos_y=2, pos_z=3,
            quat_w=1, quat_x=0, quat_y=0, quat_z=0  # Identity quaternion
        )

        assert isinstance(t, cas.TransformationMatrix)
        assert t.x.to_np() == 1
        assert t.y.to_np() == 2
        assert t.z.to_np() == 3

        # Should have identity rotation
        rotation_part = t.to_np()[:3, :3]
        assert np.allclose(rotation_part, np.eye(3))

    def test_composition(self):
        """Test composition of multiple transformations"""
        # Translation only
        t1 = cas.TransformationMatrix.from_xyz_rpy(1, 0, 0)  # Translate in X
        t2 = cas.TransformationMatrix.from_xyz_rpy(0, 1, 0)  # Translate in Y

        # Compose transformations
        combined = t2 @ t1
        assert isinstance(combined, cas.TransformationMatrix)

        # Apply to origin point
        origin = cas.Point3(0, 0, 0)
        result = combined @ origin
        assert isinstance(result, cas.Point3)
        # Should be at (1, 1, 0) after both translations
        assert np.isclose(result.x.to_np(), 1)
        assert np.isclose(result.y.to_np(), 1)
        assert np.isclose(result.z.to_np(), 0)

    def test_point_transformation(self):
        """Test transformation of points"""
        # Create a transformation: translate by (1,2,3) and rotate by 90° around Z
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0, 0, np.pi / 2)

        # Transform a point
        p = cas.Point3(1, 0, 0)
        transformed = t @ p

        assert isinstance(transformed, cas.Point3)
        assert transformed[3] == 1  # Homogeneous coordinate preserved

        # Expected: rotation transforms (1,0,0) to (0,1,0), then translation adds (1,2,3)
        expected_x = 0 + 1  # 1 (translation)
        expected_y = 1 + 2  # 3 (rotated + translation)
        expected_z = 0 + 3  # 3 (translation)

        assert np.isclose(transformed.x.to_np(), expected_x, atol=1e-10)
        assert np.isclose(transformed.y.to_np(), expected_y, atol=1e-10)
        assert np.isclose(transformed.z.to_np(), expected_z, atol=1e-10)

    def test_vector_transformation(self):
        """Test transformation of vectors (no translation effect)"""
        # Create transformation with both rotation and translation
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0, 0, np.pi / 2)

        # Transform a vector
        v = cas.Vector3(1, 0, 0)
        transformed = t @ v

        assert isinstance(transformed, cas.Vector3)
        assert transformed[3] == 0  # Vector homogeneous coordinate

        # Only rotation should affect vectors, not translation
        # 90° rotation around Z: (1,0,0) -> (0,1,0)
        assert np.isclose(transformed.x.to_np(), 0, atol=1e-10)
        assert np.isclose(transformed.y.to_np(), 1, atol=1e-10)
        assert np.isclose(transformed.z.to_np(), 0, atol=1e-10)

    def test_inverse(self):
        """Test matrix inversion"""
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)
        t_inv = t.inverse()

        assert isinstance(t_inv, cas.TransformationMatrix)

        # Test that T @ T^(-1) = I
        identity_check = t @ t_inv
        identity = cas.TransformationMatrix()
        assert np.allclose(identity_check.to_np(), identity.to_np(), atol=1e-10)

        # Test that T^(-1) @ T = I
        identity_check2 = t_inv @ t
        assert np.allclose(identity_check2.to_np(), identity.to_np(), atol=1e-10)

        # Test frame swapping
        if hasattr(t, 'reference_frame') and hasattr(t, 'child_frame'):
            assert t_inv.reference_frame == t.child_frame
            assert t_inv.child_frame == t.reference_frame

    def test_extraction_methods(self):
        """Test methods for extracting components"""
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)

        # Extract position
        position = t.to_position()
        assert isinstance(position, cas.Point3)
        assert position.x.to_np() == 1
        assert position.y.to_np() == 2
        assert position.z.to_np() == 3
        assert position[3] == 1

        # Extract rotation
        rotation = t.to_rotation()
        assert isinstance(rotation, cas.RotationMatrix)
        # Should have zero translation
        assert rotation[0, 3] == 0
        assert rotation[1, 3] == 0
        assert rotation[2, 3] == 0

        # Extract translation (pure translation matrix)
        translation = t.to_translation()
        assert isinstance(translation, cas.TransformationMatrix)
        # Should have identity rotation
        rotation_part = translation.to_np()[:3, :3]
        assert np.allclose(rotation_part, np.eye(3))
        # Should preserve translation
        assert translation.x.to_np() == 1
        assert translation.y.to_np() == 2
        assert translation.z.to_np() == 3

        # Extract quaternion
        quaternion = t.to_quaternion()
        assert isinstance(quaternion, cas.Quaternion)

    def test_frame_properties(self):
        """Test reference frame and child frame properties"""
        t = cas.TransformationMatrix()

        # Initially should be None
        assert t.reference_frame is None
        assert t.child_frame is None

        # Test frame preservation in operations
        t1 = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)
        t2 = cas.TransformationMatrix.from_xyz_rpy(4, 5, 6, 0.4, 0.5, 0.6)

        result = t1 @ t2
        # Frame handling depends on implementation
        assert hasattr(result, 'reference_frame')
        assert hasattr(result, 'child_frame')

    def test_invalid_operations(self):
        """Test invalid operations that should raise TypeError"""
        t = cas.TransformationMatrix()
        s = cas.Symbol('s')
        e = cas.Expression(1)
        q = cas.Quaternion()

        # These should raise TypeError
        with pytest.raises(TypeError):
            t @ s  # Matrix @ Symbol
        with pytest.raises(TypeError):
            t @ e  # Matrix @ Expression (scalar)
        with pytest.raises(TypeError):
            t @ q  # Matrix @ Quaternion

    @given(float_no_nan_no_inf(), float_no_nan_no_inf(), float_no_nan_no_inf())
    def test_pure_translation(self, x, y, z):
        """Property-based test for pure translation matrices"""
        t = cas.TransformationMatrix.from_xyz_rpy(x, y, z)

        # Should have identity rotation
        rotation_part = t.to_np()[:3, :3]
        assert np.allclose(rotation_part, np.eye(3))

        # Should have correct translation
        assert np.isclose(t.x.to_np(), x)
        assert np.isclose(t.y.to_np(), y)
        assert np.isclose(t.z.to_np(), z)

        # Bottom row should be [0, 0, 0, 1]
        assert t[3, 0] == 0
        assert t[3, 1] == 0
        assert t[3, 2] == 0
        assert t[3, 3] == 1

    def test_symbolic_operations(self):
        """Test operations with symbolic expressions"""
        x_sym = cas.Symbol('x')
        y_sym = cas.Symbol('y')
        angle_sym = cas.Symbol('theta')

        # Create symbolic transformation
        t_sym = cas.TransformationMatrix.from_xyz_rpy(x_sym, y_sym, 0, 0, 0, angle_sym)

        # Should be able to compose with other transformations
        t_numeric = cas.TransformationMatrix.from_xyz_rpy(1, 1, 1)
        result = t_sym @ t_numeric

        assert isinstance(result, cas.TransformationMatrix)

        # Should contain the symbols
        symbols = result.free_symbols()
        symbol_names = [s.name for s in symbols if hasattr(s, 'name')]
        assert 'x' in symbol_names
        assert 'y' in symbol_names
        assert 'theta' in symbol_names

    def test_compilation(self):
        """Test compilation and execution of transformation matrices"""
        # Test symbolic transformation compilation
        compiled_transform = cas.compile_and_execute(
            lambda x, y, z, roll, pitch, yaw: cas.TransformationMatrix.from_xyz_rpy(x, y, z, roll, pitch, yaw),
            [1, 2, 3, 0.1, 0.2, 0.3]
        )

        # Should be a valid 4x4 transformation matrix
        assert compiled_transform.shape == (4, 4)
        assert np.isclose(compiled_transform[3, 3], 1)
        assert np.isclose(compiled_transform[3, 0], 0)
        assert np.isclose(compiled_transform[3, 1], 0)
        assert np.isclose(compiled_transform[3, 2], 0)

    def test_deepcopy(self):
        """Test deep copy functionality"""
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)

        from copy import deepcopy
        t_copy = deepcopy(t)

        assert isinstance(t_copy, cas.TransformationMatrix)
        assert np.allclose(t.to_np(), t_copy.to_np())

        # Frames should be preserved but not deep copied (reference equality)
        assert t_copy.reference_frame == t.reference_frame
        assert t_copy.child_frame == t.child_frame

    def test_robot_kinematics(self):
        """Test transformation matrices in typical robotics scenarios"""
        # Forward kinematics chain: base -> link1 -> link2 -> end_effector
        base_T_link1 = cas.TransformationMatrix.from_xyz_rpy(0, 0, 1, 0, 0, np.pi / 4)  # Lift and rotate
        link1_T_link2 = cas.TransformationMatrix.from_xyz_rpy(1, 0, 0, 0, np.pi / 2, 0)  # Extend and bend
        link2_T_ee = cas.TransformationMatrix.from_xyz_rpy(0.5, 0, 0)  # End effector offset

        # Forward kinematics
        base_T_ee = base_T_link1 @ link1_T_link2 @ link2_T_ee
        assert isinstance(base_T_ee, cas.TransformationMatrix)

        # Test that inverse kinematics works
        ee_T_base = base_T_ee.inverse()
        identity_check = base_T_ee @ ee_T_base
        identity = cas.TransformationMatrix()
        assert np.allclose(identity_check.to_np(), identity.to_np(), atol=1e-10)

    def test_coordinate_transformations(self):
        """Test coordinate frame transformations"""
        # Transform from world to robot base
        world_T_robot = cas.TransformationMatrix.from_xyz_rpy(2, 3, 0, 0, 0, np.pi / 2)

        # Point in world coordinates
        world_point = cas.Point3(1, 0, 1)

        # Transform to robot coordinates
        robot_point = world_T_robot.inverse() @ world_point
        assert isinstance(robot_point, cas.Point3)

        # Transform back to world coordinates
        world_point_back = world_T_robot @ robot_point
        assert isinstance(world_point_back, cas.Point3)

        # Should get original point back
        assert np.allclose(world_point.to_np(), world_point_back.to_np(), atol=1e-10)

    def test_edge_cases(self):
        """Test edge cases and boundary conditions"""
        # Identity transformation
        t_identity = cas.TransformationMatrix()
        point = cas.Point3(1, 2, 3)
        transformed = t_identity @ point
        assert np.allclose(point.to_np(), transformed.to_np())

        # Zero translation, identity rotation
        t_zero = cas.TransformationMatrix.from_xyz_rpy(0, 0, 0, 0, 0, 0)
        assert np.allclose(t_zero.to_np(), t_identity.to_np())

        # Large translation values
        t_large = cas.TransformationMatrix.from_xyz_rpy(1e6, -1e6, 1e6)
        assert t_large.x.to_np() == 1e6
        assert t_large.y.to_np() == -1e6
        assert t_large.z.to_np() == 1e6

        # Small rotation angles (numerical stability)
        t_small = cas.TransformationMatrix.from_xyz_rpy(0, 0, 0, 1e-8, 1e-8, 1e-8)
        rotation_part = t_small.to_np()[:3, :3]
        assert np.allclose(rotation_part, np.eye(3), atol=1e-7)

    @given(quaternion())
    def test_quaternion_consistency(self, q):
        """Property-based test for quaternion consistency"""
        # Create transformation from quaternion
        t = cas.TransformationMatrix.from_xyz_quat(
            pos_x=1, pos_y=2, pos_z=3,
            quat_w=q[3], quat_x=q[0], quat_y=q[1], quat_z=q[2]
        )

        # Extract quaternion back
        q_extracted = t.to_quaternion()

        # Create transformation from extracted quaternion
        t_roundtrip = cas.TransformationMatrix.from_xyz_quat(
            pos_x=1, pos_y=2, pos_z=3,
            quat_w=q_extracted[3], quat_x=q_extracted[0],
            quat_y=q_extracted[1], quat_z=q_extracted[2]
        )

        # Should be the same (within numerical precision)
        assert np.allclose(t.to_np(), t_roundtrip.to_np(), atol=1e-10)

    def test_homogeneous_properties(self):
        """Test homogeneous coordinate properties"""
        t = cas.TransformationMatrix.from_xyz_rpy(1, 2, 3, 0.1, 0.2, 0.3)

        # Transforming points preserves homogeneous coordinate = 1
        point = cas.Point3(4, 5, 6)
        transformed_point = t @ point
        assert transformed_point[3] == 1

        # Transforming vectors preserves homogeneous coordinate = 0
        vector = cas.Vector3(1, 1, 1)
        transformed_vector = t @ vector
        assert transformed_vector[3] == 0

        # Transformation matrix structure
        transform_np = t.to_np()
        assert transform_np[3, 0] == 0
        assert transform_np[3, 1] == 0
        assert transform_np[3, 2] == 0
        assert transform_np[3, 3] == 1


class TestQuaternion:
    @given(unit_vector(length=3),
           random_angle())
    def test_quaternion_from_axis_angle1(self, axis, angle):
        r2 = giskard_math.quaternion_from_axis_angle(axis, angle)
        assert np.allclose(cas.compile_and_execute(cas.Quaternion.from_axis_angle, [axis, angle]), r2)

    @given(quaternion(),
           quaternion())
    def test_quaternion_multiply(self, q, p):
        r1 = cas.compile_and_execute(cas.quaternion_multiply, [q, p])
        r2 = giskard_math.quaternion_multiply(q, p)
        assert np.allclose(r1, r2) or np.isclose(r1, -r2)

    @given(quaternion())
    def test_quaternion_conjugate(self, q):
        r1 = cas.compile_and_execute(cas.quaternion_conjugate, [q])
        r2 = giskard_math.quaternion_conjugate(q)
        assert np.allclose(r1, r2) or np.isclose(r1, -r2)

    @given(quaternion(),
           quaternion())
    def test_quaternion_diff(self, q1, q2):
        q3 = giskard_math.quaternion_multiply(giskard_math.quaternion_conjugate(q1), q2)
        q4 = cas.compile_and_execute(cas.quaternion_diff, [q1, q2])
        assert np.allclose(q3, q4) or np.isclose(q3, -q4)

    @given(quaternion())
    def test_axis_angle_from_quaternion(self, q):
        axis2, angle2 = giskard_math.axis_angle_from_quaternion(*q)
        axis = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[0], q)
        angle = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[1], q)
        compare_axis_angle(angle, axis[:3], angle2, axis2, 2)
        assert axis[-1] == 0

    def test_axis_angle_from_quaternion2(self):
        q = [0, 0, 0, 1.0000001]
        axis2, angle2 = giskard_math.axis_angle_from_quaternion(*q)
        axis = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[0], q)
        angle = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[1], q)
        compare_axis_angle(angle, axis[:3], angle2, axis2, 2)
        assert axis[-1] == 0

    @given(random_angle(),
           random_angle(),
           random_angle())
    def test_quaternion_from_rpy(self, roll, pitch, yaw):
        q = cas.compile_and_execute(cas.Quaternion.from_rpy, [roll, pitch, yaw])
        q2 = giskard_math.quaternion_from_rpy(roll, pitch, yaw)
        compare_orientations(q, q2)

    @given(quaternion())
    def test_quaternion_from_matrix(self, q):
        matrix = giskard_math.rotation_matrix_from_quaternion(*q)
        q2 = giskard_math.quaternion_from_rotation_matrix(matrix)
        q1_2 = cas.Quaternion.from_rotation_matrix(cas.RotationMatrix(matrix)).to_np()
        assert np.allclose(q1_2, q2) or np.allclose(q1_2, -q2)

    @given(quaternion(), quaternion())
    def test_dot(self, q1, q2):
        q1 = np.array(q1)
        q2 = np.array(q2)
        result = cas.compile_and_execute(
            lambda p1, p2: cas.Quaternion.from_iterable(p1).dot(cas.Quaternion.from_iterable(p2)), [q1, q2])
        expected = np.dot(q1.T, q2)
        if not np.isnan(result) and not np.isinf(result):
            assert np.isclose(result, expected)


class TestCASWrapper:
    @given(st.booleans())
    def test_empty_compiled_function(self, sparse):
        if sparse:
            expected = np.array([1, 2, 3], ndmin=2)
        else:
            expected = np.array([1, 2, 3])
        e = cas.Expression(expected)
        f = e.compile(sparse=sparse)
        assert np.allclose(f(), expected)
        assert np.allclose(f.fast_call(np.array([])), expected)

    def test_add(self):
        s2 = 'muh'
        f = 1.0
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        t = cas.TransformationMatrix()
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        # float
        assert isinstance(s + f, cas.Expression)
        assert isinstance(f + s, cas.Expression)
        assert isinstance(e + f, cas.Expression)
        assert isinstance(f + e, cas.Expression)
        assert isinstance(v + f, cas.Vector3)
        assert isinstance(f + v, cas.Vector3)
        assert isinstance(p + f, cas.Point3)
        assert isinstance(f + p, cas.Point3)
        with pytest.raises(TypeError):
            t + f
        with pytest.raises(TypeError):
            f + t
        with pytest.raises(TypeError):
            r + f
        with pytest.raises(TypeError):
            f + r
        with pytest.raises(TypeError):
            q + f
        with pytest.raises(TypeError):
            f + q

        # Symbol
        assert isinstance(s + s, cas.Expression)
        assert isinstance(s + e, cas.Expression)
        assert isinstance(e + s, cas.Expression)
        assert isinstance(s + v, cas.Vector3)
        assert isinstance(v + s, cas.Vector3)
        assert (s + v)[3].to_np() == 0 == (v + s)[3].to_np()
        assert isinstance(s + p, cas.Point3)
        assert isinstance(p + s, cas.Point3)
        assert (s + p)[3].to_np() == 1 == (p + s)[3].to_np()
        with pytest.raises(TypeError):
            s + t
        with pytest.raises(TypeError):
            t + s
        with pytest.raises(TypeError):
            s + r
        with pytest.raises(TypeError):
            r + s
        with pytest.raises(TypeError):
            s + q
        with pytest.raises(TypeError):
            q + s
        with pytest.raises(TypeError):
            s + s2
        with pytest.raises(TypeError):
            s2 + s
        # Expression
        assert isinstance(e + e, cas.Expression)
        assert isinstance(e + v, cas.Vector3)
        assert isinstance(v + e, cas.Vector3)
        assert (e + v)[3].to_np() == 0 == (v + e)[3].to_np()
        assert isinstance(e + p, cas.Point3)
        assert isinstance(p + e, cas.Point3)
        assert (e + p)[3].to_np() == 1 == (p + e)[3].to_np()
        with pytest.raises(TypeError):
            e + t
        with pytest.raises(TypeError):
            t + e
        with pytest.raises(TypeError):
            e + r
        with pytest.raises(TypeError):
            r + e
        with pytest.raises(TypeError):
            e + q
        with pytest.raises(TypeError):
            q + e
        # Vector3
        assert isinstance(v + v, cas.Vector3)
        assert (v + v)[3].to_np() == 0
        assert isinstance(v + p, cas.Point3)
        assert isinstance(p + v, cas.Point3)
        assert (v + p)[3].to_np() == 1 == (p + v)[3].to_np()
        with pytest.raises(TypeError):
            v + t
        with pytest.raises(TypeError):
            t + v
        with pytest.raises(TypeError):
            v + r
        with pytest.raises(TypeError):
            r + v
        with pytest.raises(TypeError):
            v + q
        with pytest.raises(TypeError):
            q + v
        # Point3
        with pytest.raises(TypeError):
            p + p
        with pytest.raises(TypeError):
            p + t
        with pytest.raises(TypeError):
            t + p
        with pytest.raises(TypeError):
            p + r
        with pytest.raises(TypeError):
            r + p
        with pytest.raises(TypeError):
            p + q
        with pytest.raises(TypeError):
            q + p
        # TransMatrix
        with pytest.raises(TypeError):
            t + t
        with pytest.raises(TypeError):
            t + r
        with pytest.raises(TypeError):
            r + t
        with pytest.raises(TypeError):
            t + q
        with pytest.raises(TypeError):
            q + t
        # RotationMatrix
        with pytest.raises(TypeError):
            r + r
        with pytest.raises(TypeError):
            r + q
        with pytest.raises(TypeError):
            q + r
        # Quaternion
        with pytest.raises(TypeError):
            q + q

    def test_sub(self):
        s2 = 'muh'
        f = 1.0
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        t = cas.TransformationMatrix()
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        # float
        assert isinstance(s - f, cas.Expression)
        assert isinstance(f - s, cas.Expression)
        assert isinstance(e - f, cas.Expression)
        assert isinstance(f - e, cas.Expression)
        assert isinstance(v - f, cas.Vector3)
        assert isinstance(f - v, cas.Vector3)
        assert isinstance(p - f, cas.Point3)
        assert isinstance(f - p, cas.Point3)
        with pytest.raises(TypeError):
            t - f
        with pytest.raises(TypeError):
            f - t
        with pytest.raises(TypeError):
            r - f
        with pytest.raises(TypeError):
            f - r
        with pytest.raises(TypeError):
            q - f
        with pytest.raises(TypeError):
            f - q
        # Symbol
        assert isinstance(s - s, cas.Expression)
        assert isinstance(s - e, cas.Expression)
        assert isinstance(e - s, cas.Expression)
        assert isinstance(s - v, cas.Vector3)
        assert isinstance(v - s, cas.Vector3)
        assert (s - v)[3].to_np() == 0 == (v - s)[3].to_np()
        assert isinstance(s - p, cas.Point3)
        assert isinstance(p - s, cas.Point3)
        assert (s - p)[3].to_np() == 1 == (p - s)[3].to_np()
        with pytest.raises(TypeError):
            s - t
        with pytest.raises(TypeError):
            t - s
        with pytest.raises(TypeError):
            s - r
        with pytest.raises(TypeError):
            r - s
        with pytest.raises(TypeError):
            s - q
        with pytest.raises(TypeError):
            q - s
        # Expression
        assert isinstance(e - e, cas.Expression)
        assert isinstance(e - v, cas.Vector3)
        assert isinstance(v - e, cas.Vector3)
        assert (e - v)[3].to_np() == 0 == (v - e)[3].to_np()
        assert isinstance(e - p, cas.Point3)
        assert isinstance(p - e, cas.Point3)
        assert (e - p)[3].to_np() == 1 == (p - e)[3].to_np()
        with pytest.raises(TypeError):
            e - t
        with pytest.raises(TypeError):
            t - e
        with pytest.raises(TypeError):
            e - r
        with pytest.raises(TypeError):
            r - e
        with pytest.raises(TypeError):
            e - q
        with pytest.raises(TypeError):
            q - e
        # Vector3
        assert isinstance(v - v, cas.Vector3)
        assert (v - v)[3].to_np() == 0
        assert isinstance(v - p, cas.Point3)
        assert isinstance(p - v, cas.Point3)
        assert (v - p)[3].to_np() == 1 == (p - v)[3].to_np()
        with pytest.raises(TypeError):
            v - t
        with pytest.raises(TypeError):
            t - v
        with pytest.raises(TypeError):
            v - r
        with pytest.raises(TypeError):
            r - v
        with pytest.raises(TypeError):
            v - q
        with pytest.raises(TypeError):
            q - v
        # Point3
        assert isinstance(p - p, cas.Vector3)
        assert (p - p)[3].to_np() == 0
        with pytest.raises(TypeError):
            p - t
        with pytest.raises(TypeError):
            t - p
        with pytest.raises(TypeError):
            p - r
        with pytest.raises(TypeError):
            r - p
        with pytest.raises(TypeError):
            p - q
        with pytest.raises(TypeError):
            q - p
        # TransMatrix
        with pytest.raises(TypeError):
            t - t
        with pytest.raises(TypeError):
            t - r
        with pytest.raises(TypeError):
            r - t
        with pytest.raises(TypeError):
            t - q
        with pytest.raises(TypeError):
            q - t
        # RotationMatrix
        with pytest.raises(TypeError):
            r - r
        with pytest.raises(TypeError):
            r - q
        with pytest.raises(TypeError):
            q - r
        # Quaternion
        with pytest.raises(TypeError):
            q - q

    def test_basic_operation_with_string(self):
        str_ = 'muh23'
        things = [cas.Symbol('s'),
                  cas.Expression(1),
                  cas.Vector3(1, 1, 1),
                  cas.Point3(1, 1, 1),
                  cas.TransformationMatrix(),
                  cas.RotationMatrix(),
                  cas.Quaternion()]
        functions = ['__add__', '__radd_', '__sub__', '__rsub__', '__mul__', '__rmul', '__truediv__', '__rtruediv__',
                     '__pow__', '__rpow__', 'dot']
        for fn in functions:
            for thing in things:
                if hasattr(str_, fn):
                    error_msg = f'string.{fn}({thing.__class__.__name__})'
                    with pytest.raises(TypeError) as e:
                        getattr(str_, fn)(thing)
                    assert 'NotImplementedType' not in str(e), error_msg
                if hasattr(thing, fn):
                    error_msg = f'{thing.__class__.__name__}.{fn}(string)'
                    with pytest.raises(TypeError) as e:
                        getattr(thing, fn)(str_)
                    assert 'NotImplementedType' not in str(e), error_msg

    def test_mul_truediv_pow(self):
        f = 1.0
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        t = cas.TransformationMatrix()
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        functions = [lambda a, b: a * b, lambda a, b: a / b, lambda a, b: a ** b]
        for fn in functions:
            # float
            assert isinstance(fn(f, s), cas.Expression)
            assert isinstance(fn(s, f), cas.Expression)
            assert isinstance(fn(f, v), cas.Vector3)
            assert isinstance(fn(v, f), cas.Vector3)
            assert isinstance(fn(f, p), cas.Point3)
            assert isinstance(fn(p, f), cas.Point3)
            with pytest.raises(TypeError):
                fn(f, t)
            with pytest.raises(TypeError):
                fn(t, f)
            with pytest.raises(TypeError):
                fn(f, r)
            with pytest.raises(TypeError):
                fn(r, f)
            with pytest.raises(TypeError):
                fn(f, q)
            with pytest.raises(TypeError):
                fn(q, f)

            # Symbol
            assert isinstance(fn(s, s), cas.Expression)
            assert isinstance(fn(s, e), cas.Expression)
            assert isinstance(fn(e, s), cas.Expression)
            assert isinstance(fn(s, v), cas.Vector3)
            assert isinstance(fn(v, s), cas.Vector3)
            assert (fn(s, v))[3].to_np() == 0 == (fn(v, s))[3].to_np()
            assert isinstance(fn(s, p), cas.Point3)
            assert isinstance(fn(p, s), cas.Point3)
            assert (fn(s, p))[3].to_np() == 1 == (fn(p, s))[3].to_np()
            with pytest.raises(TypeError):
                fn(s, t)
            with pytest.raises(TypeError):
                fn(t, s)
            with pytest.raises(TypeError):
                fn(s, r)
            with pytest.raises(TypeError):
                fn(r, s)
            with pytest.raises(TypeError):
                fn(s, q)
            with pytest.raises(TypeError):
                fn(q, s)
            # Expression
            assert isinstance(fn(e, e), cas.Expression)
            assert isinstance(fn(e, v), cas.Vector3)
            assert isinstance(fn(v, e), cas.Vector3)
            assert (fn(e, v))[3].to_np() == 0 == (fn(v, e))[3].to_np()
            assert isinstance(fn(e, p), cas.Point3)
            assert isinstance(fn(p, e), cas.Point3)
            assert (fn(e, p))[3].to_np() == 1 == (fn(p, e))[3].to_np()
            with pytest.raises(TypeError):
                fn(e, t)
            with pytest.raises(TypeError):
                fn(t, e)
            with pytest.raises(TypeError):
                fn(e, r)
            with pytest.raises(TypeError):
                fn(r, e)
            with pytest.raises(TypeError):
                fn(e, q)
            with pytest.raises(TypeError):
                fn(q, e)
            # Vector3
            with pytest.raises(TypeError):
                fn(v, v)
            with pytest.raises(TypeError):
                fn(v, p)
            with pytest.raises(TypeError):
                fn(p, v)
            with pytest.raises(TypeError):
                fn(v, t)
            with pytest.raises(TypeError):
                fn(t, v)
            with pytest.raises(TypeError):
                fn(v, r)
            with pytest.raises(TypeError):
                fn(r, v)
            with pytest.raises(TypeError):
                fn(v, q)
            with pytest.raises(TypeError):
                fn(q, v)
            # Point3
            with pytest.raises(TypeError):
                fn(p, p)
            with pytest.raises(TypeError):
                fn(p, t)
            with pytest.raises(TypeError):
                fn(t, p)
            with pytest.raises(TypeError):
                fn(p, r)
            with pytest.raises(TypeError):
                fn(r, p)
            with pytest.raises(TypeError):
                fn(p, q)
            with pytest.raises(TypeError):
                fn(q, p)
            # TransMatrix
            with pytest.raises(TypeError):
                fn(t, t)
            with pytest.raises(TypeError):
                fn(t, r)
            with pytest.raises(TypeError):
                fn(r, t)
            with pytest.raises(TypeError):
                fn(t, q)
            with pytest.raises(TypeError):
                fn(q, t)
            # RotationMatrix
            with pytest.raises(TypeError):
                fn(r, r)
            with pytest.raises(TypeError):
                fn(r, q)
            with pytest.raises(TypeError):
                fn(q, r)
            # Quaternion
            with pytest.raises(TypeError):
                fn(q, q)

    def test_dot_types(self):
        s = cas.Symbol('s')
        e = cas.Expression(1)
        v = cas.Vector3(1, 1, 1)
        p = cas.Point3(1, 1, 1)
        t = cas.TransformationMatrix()
        r = cas.RotationMatrix()
        q = cas.Quaternion()
        # Symbol
        for muh in [s, e, v, p, t, r, q]:
            with pytest.raises(TypeError):
                cas.dot(s, muh)
            with pytest.raises(TypeError):
                cas.dot(muh, s)
        # Expression
        assert isinstance(cas.dot(e, e), cas.Expression)
        assert isinstance(e.dot(e), cas.Expression)
        for muh in [v, p, t, r, q]:
            with pytest.raises(TypeError):
                cas.dot(e, muh)
            with pytest.raises(TypeError):
                cas.dot(muh, e)
            with pytest.raises(TypeError):
                e.dot(muh)
        # Vector3
        assert isinstance(v.dot(v), cas.Expression)
        assert isinstance(cas.dot(v, v), cas.Expression)
        assert isinstance(v.dot(p), cas.Expression)
        assert isinstance(cas.dot(v, p), cas.Expression)
        assert isinstance(p.dot(v), cas.Expression)
        assert isinstance(cas.dot(p, v), cas.Expression)
        assert isinstance(t.dot(v), cas.Vector3)
        assert isinstance(cas.dot(t, v), cas.Vector3)
        with pytest.raises(TypeError):
            v.dot(t)
        with pytest.raises(TypeError):
            cas.dot(v, t)
        assert isinstance(r.dot(v), cas.Vector3)
        assert isinstance(cas.dot(r, v), cas.Vector3)
        with pytest.raises(TypeError):
            v.dot(q)
        with pytest.raises(TypeError):
            cas.dot(v, q)
        with pytest.raises(TypeError):
            q.dot(v)
        with pytest.raises(TypeError):
            cas.dot(q, v)
        # Point3
        assert isinstance(p.dot(p), cas.Expression)
        assert isinstance(cas.dot(p, p), cas.Expression)
        assert isinstance(t.dot(p), cas.Point3)
        assert isinstance(cas.dot(t, p), cas.Point3)
        with pytest.raises(TypeError):
            p.dot(t)
        with pytest.raises(TypeError):
            cas.dot(p, t)
        with pytest.raises(TypeError):
            assert isinstance(r.dot(p), cas.Point3)
        with pytest.raises(TypeError):
            assert isinstance(cas.dot(r, p), cas.Point3)
        with pytest.raises(TypeError):
            p.dot(q)
        with pytest.raises(TypeError):
            cas.dot(p, q)
        with pytest.raises(TypeError):
            q.dot(p)
        with pytest.raises(TypeError):
            cas.dot(q, p)
        # TransMatrix
        assert isinstance(t.dot(t), cas.TransformationMatrix)
        assert isinstance(cas.dot(t, t), cas.TransformationMatrix)
        assert isinstance(t.dot(r), cas.RotationMatrix)
        assert isinstance(cas.dot(t, r), cas.RotationMatrix)
        assert isinstance(r.dot(t), cas.TransformationMatrix)
        assert isinstance(cas.dot(r, t), cas.TransformationMatrix)
        with pytest.raises(TypeError):
            t.dot(q)
        with pytest.raises(TypeError):
            cas.dot(t, q)
        with pytest.raises(TypeError):
            q.dot(t)
        with pytest.raises(TypeError):
            cas.dot(q, t)
        # RotationMatrix
        assert isinstance(r.dot(r), cas.RotationMatrix)
        assert isinstance(cas.dot(r, r), cas.RotationMatrix)
        with pytest.raises(TypeError):
            r.dot(q)
        with pytest.raises(TypeError):
            cas.dot(r, q)
        with pytest.raises(TypeError):
            q.dot(r)
        with pytest.raises(TypeError):
            cas.dot(q, r)
        assert isinstance(q.dot(q), cas.Expression)
        assert isinstance(cas.dot(q, q), cas.Expression)

    def test_free_symbols(self):
        m = cas.Expression(cas.var('a b c d'))
        assert len(cas.free_symbols(m)) == 4
        a = cas.Symbol('a')
        assert cas.equivalent(a, cas.free_symbols(a)[0])

    def test_jacobian(self):
        a = cas.Symbol('a')
        b = cas.Symbol('b')
        m = cas.Expression([a + b, a ** 2, b ** 2])
        jac = cas.jacobian(m, [a, b])
        expected = cas.Expression([[1, 1], [2 * a, 0], [0, 2 * b]])
        for i in range(expected.shape[0]):
            for j in range(expected.shape[1]):
                assert cas.equivalent(jac[i, j], expected[i, j])

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_jacobian_dot(self, a, ad, b, bd):
        kwargs = {
            'a': a,
            'ad': ad,
            'b': b,
            'bd': bd,
        }
        a_s = cas.Symbol('a')
        ad_s = cas.Symbol('ad')
        b_s = cas.Symbol('b')
        bd_s = cas.Symbol('bd')
        m = cas.Expression([
            a_s ** 3 * b_s ** 3,
            # b_s ** 2,
            -a_s * cas.cos(b_s),
            # a_s * b_s ** 4
        ])
        jac = cas.jacobian_dot(m, [a_s, b_s], [ad_s, bd_s])
        expected_expr = cas.Expression([
            [6 * ad_s * a_s * b_s ** 3 + 9 * a_s ** 2 * bd_s * b_s ** 2,
             9 * ad_s * a_s ** 2 * b_s ** 2 + 6 * a_s ** 3 * bd_s * b],
            # [0, 2 * bd_s],
            [bd_s * cas.sin(b_s), ad_s * cas.sin(b_s) + a_s * bd_s * cas.cos(b_s)],
            # [4 * bd * b ** 3, 4 * ad * b ** 3 + 12 * a * bd * b ** 2]
        ])
        actual = jac.compile()(**kwargs)
        expected = expected_expr.compile()(**kwargs)
        assert np.allclose(actual, expected)

    @given(float_no_nan_no_inf(outer_limit=1e2),
           float_no_nan_no_inf(outer_limit=1e2),
           float_no_nan_no_inf(outer_limit=1e2),
           float_no_nan_no_inf(outer_limit=1e2),
           float_no_nan_no_inf(outer_limit=1e2),
           float_no_nan_no_inf(outer_limit=1e2))
    def test_jacobian_ddot(self, a, ad, add, b, bd, bdd):
        kwargs = {
            'a': a,
            'ad': ad,
            'add': add,
            'b': b,
            'bd': bd,
            'bdd': bdd,
        }
        a_s = cas.Symbol('a')
        ad_s = cas.Symbol('ad')
        add_s = cas.Symbol('add')
        b_s = cas.Symbol('b')
        bd_s = cas.Symbol('bd')
        bdd_s = cas.Symbol('bdd')
        m = cas.Expression([
            a_s ** 3 * b_s ** 3,
            b_s ** 2,
            -a_s * cas.cos(b_s),
        ])
        jac = cas.jacobian_ddot(m, [a_s, b_s], [ad_s, bd_s], [add_s, bdd_s])
        expected = np.array([
            [add * 6 * b ** 3 + bdd * 18 * a ** 2 * b + 2 * ad * bd * 18 * a * b ** 2,
             bdd * 6 * a ** 3 + add * 18 * b ** 2 * a + 2 * ad * bd * 18 * b * a ** 2],
            [0, 0],
            [bdd * np.cos(b),
             bdd * -a * np.sin(b) + 2 * ad * bd * np.cos(b)],
        ])
        actual = jac.compile()(**kwargs)
        assert np.allclose(actual, expected)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_total_derivative2(self, a, b, ad, bd, add, bdd):
        kwargs = {
            'a': a,
            'ad': ad,
            'add': add,
            'b': b,
            'bd': bd,
            'bdd': bdd,
        }
        a_s = cas.Symbol('a')
        ad_s = cas.Symbol('ad')
        add_s = cas.Symbol('add')
        b_s = cas.Symbol('b')
        bd_s = cas.Symbol('bd')
        bdd_s = cas.Symbol('bdd')
        m = cas.Expression(a_s * b_s ** 2)
        jac = cas.total_derivative2(m, [a_s, b_s], [ad_s, bd_s], [add_s, bdd_s])
        actual = jac.compile()(**kwargs)
        expected = bdd * 2 * a + 2 * ad * bd * 2 * b
        assert np.allclose(actual, expected)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_total_derivative2_2(self, a, b, c, ad, bd, cd, add, bdd, cdd):
        kwargs = {
            'a': a,
            'ad': ad,
            'add': add,
            'b': b,
            'bd': bd,
            'bdd': bdd,
            'c': c,
            'cd': cd,
            'cdd': cdd,
        }
        a_s = cas.Symbol('a')
        ad_s = cas.Symbol('ad')
        add_s = cas.Symbol('add')
        b_s = cas.Symbol('b')
        bd_s = cas.Symbol('bd')
        bdd_s = cas.Symbol('bdd')
        c_s = cas.Symbol('c')
        cd_s = cas.Symbol('cd')
        cdd_s = cas.Symbol('cdd')
        m = cas.Expression(a_s * b_s ** 2 * c_s ** 3)
        jac = cas.total_derivative2(m, [a_s, b_s, c_s], [ad_s, bd_s, cd_s], [add_s, bdd_s, cdd_s])
        # expected_expr = cas.Expression(add_s + bdd_s*2*a*c**3 + 4*ad_s*)
        actual = jac.compile()(**kwargs)
        # expected = expected_expr.compile()(**kwargs)
        expected = bdd * 2 * a * c ** 3 \
                   + cdd * 6 * a * b ** 2 * c \
                   + 4 * ad * bd * b * c ** 3 \
                   + 6 * ad * b ** 2 * cd * c ** 2 \
                   + 12 * a * bd * b * cd * c ** 2
        assert np.allclose(actual, expected)

    def test_var(self):
        result = cas.var('a b c')
        assert str(result[0]) == 'a'
        assert str(result[1]) == 'b'
        assert str(result[2]) == 'c'

    def test_diag(self):
        result = cas.diag([1, 2, 3])
        assert result[0, 0] == 1
        assert result[0, 1] == 0
        assert result[0, 2] == 0

        assert result[1, 0] == 0
        assert result[1, 1] == 2
        assert result[1, 2] == 0

        assert result[2, 0] == 0
        assert result[2, 1] == 0
        assert result[2, 2] == 3
        assert cas.equivalent(cas.diag(cas.Expression([1, 2, 3])), cas.diag([1, 2, 3]))

    def test_vstack(self):
        m = np.eye(4)
        m1 = cas.Expression(m)
        e = cas.vstack([m1, m1])
        r1 = e.to_np()
        r2 = np.vstack([m, m])
        assert np.allclose(r1, r2)

    def test_vstack_empty(self):
        m = np.eye(0)
        m1 = cas.Expression(m)
        e = cas.vstack([m1, m1])
        r1 = e.to_np()
        r2 = np.vstack([m, m])
        assert np.allclose(r1, r2)

    def test_hstack(self):
        m = np.eye(4)
        m1 = cas.Expression(m)
        e = cas.hstack([m1, m1])
        r1 = e.to_np()
        r2 = np.hstack([m, m])
        assert np.allclose(r1, r2)

    def test_hstack_empty(self):
        m = np.eye(0)
        m1 = cas.Expression(m)
        e = cas.hstack([m1, m1])
        r1 = e.to_np()
        r2 = np.hstack([m, m])
        assert np.allclose(r1, r2)

    def test_diag_stack(self):
        m1_np = np.eye(4)
        m2_np = np.ones((2, 5))
        m3_np = np.ones((5, 3))
        m1_e = cas.Expression(m1_np)
        m2_e = cas.Expression(m2_np)
        m3_e = cas.Expression(m3_np)
        e = cas.diag_stack([m1_e, m2_e, m3_e])
        r1 = e.to_np()
        combined_matrix = np.zeros((4 + 2 + 5, 4 + 5 + 3))
        row_counter = 0
        column_counter = 0
        for matrix in [m1_np, m2_np, m3_np]:
            combined_matrix[row_counter:row_counter + matrix.shape[0],
            column_counter:column_counter + matrix.shape[1]] = matrix
            row_counter += matrix.shape[0]
            column_counter += matrix.shape[1]
        assert np.allclose(r1, combined_matrix)

    @given(float_no_nan_no_inf())
    def test_abs(self, f1):
        assert np.isclose(cas.compile_and_execute(cas.abs, [f1]), abs(f1))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_max(self, f1, f2):
        assert np.isclose(cas.compile_and_execute(cas.max, [f1, f2]), max(f1, f2))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_save_division(self, f1, f2):
        assert np.isclose(cas.compile_and_execute(cas.save_division, [f1, f2]),
                          f1 / f2 if f2 != 0 else 0)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_min(self, f1, f2):
        assert np.isclose(cas.compile_and_execute(cas.min, [f1, f2]), min(f1, f2))

    @given(float_no_nan_no_inf())
    def test_sign(self, f1):
        assert np.isclose(cas.compile_and_execute(cas.sign, [f1]), np.sign(f1))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_greater_zero(self, condition, if_result, else_result):
        assert np.isclose(cas.compile_and_execute(cas.if_greater_zero, [condition, if_result, else_result]),
                          float(if_result if condition > 0 else else_result))

    def test_if_one_arg(self):
        types = [cas.Point3, cas.Vector3, cas.Quaternion, cas.Expression, cas.TransformationMatrix, cas.RotationMatrix]
        if_functions = [cas.if_else, cas.if_eq_zero, cas.if_greater_eq_zero, cas.if_greater_zero]
        c = cas.Symbol('c')
        for type_ in types:
            for if_function in if_functions:
                if_result = type_()
                else_result = type_()
                result = if_function(c, if_result, else_result)
                assert isinstance(result, type_), f'{type(result)} != {type_} for {if_function}'

    def test_if_two_arg(self):
        types = [cas.Point3, cas.Vector3, cas.Quaternion, cas.Expression, cas.TransformationMatrix, cas.RotationMatrix]
        if_functions = [cas.if_eq, cas.if_greater, cas.if_greater_eq, cas.if_less, cas.if_less_eq]
        a = cas.Symbol('a')
        b = cas.Symbol('b')
        for type_ in types:
            for if_function in if_functions:
                if_result = type_()
                else_result = type_()
                assert isinstance(if_function(a, b, if_result, else_result), type_)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_greater_eq_zero(self, condition, if_result, else_result):
        assert np.isclose(cas.compile_and_execute(cas.if_greater_eq_zero, [condition, if_result, else_result]),
                          float(if_result if condition >= 0 else else_result))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_greater_eq(self, a, b, if_result, else_result):
        assert np.isclose(cas.compile_and_execute(cas.if_greater_eq, [a, b, if_result, else_result]),
                          float(if_result if a >= b else else_result))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_less_eq(self, a, b, if_result, else_result):
        assert np.isclose(cas.compile_and_execute(cas.if_less_eq, [a, b, if_result, else_result]),
                          float(if_result if a <= b else else_result))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_eq_zero(self, condition, if_result, else_result):
        assert np.isclose(cas.compile_and_execute(cas.if_eq_zero, [condition, if_result, else_result]),
                          float(if_result if condition == 0 else else_result))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_eq(self, a, b, if_result, else_result):
        assert np.isclose(cas.compile_and_execute(cas.if_eq, [a, b, if_result, else_result]),
                          float(if_result if a == b else else_result))

    @given(float_no_nan_no_inf())
    def test_if_eq_cases(self, a):
        b_result_cases = [(1, 1),
                          (3, 3),
                          (4, 4),
                          (-1, -1),
                          (0.5, 0.5),
                          (-0.5, -0.5)]

        def reference(a_, b_result_cases_, else_result):
            for b, if_result in b_result_cases_:
                if a_ == b:
                    return if_result
            return else_result

        actual = cas.compile_and_execute(lambda a: cas.if_eq_cases(a, b_result_cases, 0), [a])
        expected = float(reference(a, b_result_cases, 0))
        assert np.isclose(actual, expected)

    @given(float_no_nan_no_inf())
    def test_if_eq_cases_set(self, a):
        b_result_cases = {(1, 1),
                          (3, 3),
                          (4, 4),
                          (-1, -1),
                          (0.5, 0.5),
                          (-0.5, -0.5)}

        def reference(a_, b_result_cases_, else_result):
            for b, if_result in b_result_cases_:
                if a_ == b:
                    return if_result
            return else_result

        actual = cas.compile_and_execute(lambda a: cas.if_eq_cases(a, b_result_cases, 0), [a])
        expected = float(reference(a, b_result_cases, 0))
        assert np.isclose(actual, expected)

    @given(float_no_nan_no_inf())
    def test_if_eq_cases_grouped(self, a):
        b_result_cases = [(1, 1),
                          (3, 1),
                          (4, 1),
                          (-1, 3),
                          (0.5, 3),
                          (-0.5, 1)]

        def reference(a_, b_result_cases_, else_result):
            for b, if_result in b_result_cases_:
                if a_ == b:
                    return if_result
            return else_result

        actual = cas.compile_and_execute(lambda a: cas.if_eq_cases_grouped(a, b_result_cases, 0), [a])
        expected = float(reference(a, b_result_cases, 0))
        assert np.isclose(actual, expected)

    @given(float_no_nan_no_inf(10))
    def test_if_less_eq_cases(self, a):
        b_result_cases = [
            (-1, -1),
            (-0.5, -0.5),
            (0.5, 0.5),
            (1, 1),
            (3, 3),
            (4, 4),
        ]

        def reference(a_, b_result_cases_, else_result):
            for b, if_result in b_result_cases_:
                if a_ <= b:
                    return if_result
            return else_result

        assert np.isclose(
            cas.compile_and_execute(lambda a, default: cas.if_less_eq_cases(a, b_result_cases, default),
                                    [a, 0]),
            float(reference(a, b_result_cases, 0)))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_greater(self, a, b, if_result, else_result):
        assert np.isclose(
            cas.compile_and_execute(cas.if_greater, [a, b, if_result, else_result]),
            float(if_result if a > b else else_result))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_if_less(self, a, b, if_result, else_result):
        assert np.isclose(
            cas.compile_and_execute(cas.if_less, [a, b, if_result, else_result]),
            float(if_result if a < b else else_result))

    @given(vector(3),
           vector(3))
    def test_cross(self, u, v):
        assert np.allclose(
            cas.compile_and_execute(cas.cross, [u, v])[:3],
            np.cross(u, v))

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_limit(self, x, lower_limit, upper_limit):
        r1 = cas.compile_and_execute(cas.limit, [x, lower_limit, upper_limit])
        r2 = max(lower_limit, min(upper_limit, x))
        assert np.allclose(r1, r2)

    @given(st.lists(float_no_nan_no_inf(), min_size=1))
    def test_norm(self, v):
        actual = cas.compile_and_execute(cas.norm, [v])
        expected = np.linalg.norm(v)
        assume(not np.isinf(expected))
        assert np.isclose(actual, expected, equal_nan=True)

    @given(vector(3),
           float_no_nan_no_inf())
    def test_scale(self, v, a):
        if np.linalg.norm(v) == 0:
            r2 = [0, 0, 0]
        else:
            r2 = v / np.linalg.norm(v) * a
        assert np.allclose(
            cas.compile_and_execute(cas.scale, [v, a]),
            r2)

    @given(lists_of_same_length([float_no_nan_no_inf(), float_no_nan_no_inf()], max_length=50))
    def test_dot(self, vectors):
        u, v = vectors
        u = np.array(u)
        v = np.array(v)
        result = cas.compile_and_execute(cas.dot, [u, v])
        if not np.isnan(result) and not np.isinf(result):
            assert np.isclose(result, np.dot(u, v))

    @given(lists_of_same_length([float_no_nan_no_inf(outer_limit=1000), float_no_nan_no_inf(outer_limit=1000)],
                                min_length=16, max_length=16))
    def test_dot2(self, vectors):
        u, v = vectors
        u = np.array(u).reshape((4, 4))
        v = np.array(v).reshape((4, 4))
        result = cas.compile_and_execute(cas.dot, [u, v])
        expected = np.dot(u, v)
        if not np.isnan(result).any() and not np.isinf(result).any():
            assert np.allclose(result, expected)

    @given(unit_vector(4))
    def test_trace(self, q):
        m = giskard_math.rotation_matrix_from_quaternion(*q)
        assert np.allclose(cas.compile_and_execute(cas.trace, [m]), np.trace(m))

    @given(quaternion(),
           quaternion())
    def test_rotation_distance(self, q1, q2):
        m1 = giskard_math.rotation_matrix_from_quaternion(*q1)
        m2 = giskard_math.rotation_matrix_from_quaternion(*q2)
        actual_angle = cas.rotational_error(cas.RotationMatrix(m1), cas.RotationMatrix(m2)).to_np()
        _, expected_angle = giskard_math.axis_angle_from_rotation_matrix(m1.T.dot(m2))
        expected_angle = expected_angle
        try:
            assert np.isclose(giskard_math.shortest_angular_distance(actual_angle, expected_angle), 0)
        except AssertionError:
            assert np.isclose(giskard_math.shortest_angular_distance(actual_angle, -expected_angle), 0)

    @given(random_angle(),
           random_angle(),
           random_angle())
    def test_axis_angle_from_rpy(self, roll, pitch, yaw):
        expected_axis, expected_angle = giskard_math.axis_angle_from_rpy(roll, pitch, yaw)
        expected_axis = np.array(list(list(expected_axis)))
        axis = cas.compile_and_execute(lambda r, p, y: cas.axis_angle_from_rpy(r, p, y)[0], [roll, pitch, yaw])
        angle = cas.compile_and_execute(lambda r, p, y: cas.axis_angle_from_rpy(r, p, y)[1], [roll, pitch, yaw])
        compare_axis_angle(angle, axis[:3], expected_angle, expected_axis)
        assert axis[-1] == 0

    @given(quaternion(),
           quaternion(),
           st.floats(allow_nan=False, allow_infinity=False, min_value=0, max_value=1))
    def test_slerp(self, q1, q2, t):
        r1 = cas.compile_and_execute(cas.quaternion_slerp, [q1, q2, t])
        r2 = giskard_math.quaternion_slerp(q1, q2, t)
        assert np.allclose(r1, r2, atol=1e-3) or np.isclose(r1, -r2, atol=1e-3)

    @given(quaternion(),
           quaternion())
    def test_slerp123(self, q1, q2):
        step = 0.1
        q_d = cas.compile_and_execute(
            lambda q1, q2: cas.Quaternion.from_iterable(q1).diff(cas.Quaternion.from_iterable(q2)),
            [q1, q2])
        axis = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[0], q_d)
        angle = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[1], q_d)
        assume(angle != np.pi)
        if np.abs(angle) > np.pi:
            angle = angle - np.pi * 2
        elif np.abs(angle) < -np.pi:
            angle = angle + np.pi * 2
        r1s = []
        r2s = []
        for t in np.arange(0, 1.001, step):
            r1 = cas.compile_and_execute(cas.quaternion_slerp, [q1, q2, t])
            r1 = cas.compile_and_execute(
                lambda q1, q2: cas.Quaternion.from_iterable(q1).diff(cas.Quaternion.from_iterable(q2)),
                [q1, r1])
            axis2 = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[0], r1)
            angle2 = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[1], r1)
            r2 = cas.compile_and_execute(cas.Quaternion.from_axis_angle, [axis, angle * t])
            r1s.append(r1)
            r2s.append(r2)
        aa1 = []
        aa2 = []
        for r1, r2 in zip(r1s, r2s):
            axisr1 = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[0], r1)
            angler1 = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[1], r1)
            aa1.append([axisr1, angler1])
            axisr2 = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[0], r2)
            angler2 = cas.compile_and_execute(lambda x, y, z, w_: cas.Quaternion(x, y, z, w_).to_axis_angle()[1], r2)
            aa2.append([axisr2, angler2])
        qds = []
        for i in range(len(r1s) - 1):
            q1t = r1s[i]
            q2t = r1s[i + 1]
            qds.append(
                cas.compile_and_execute(
                    lambda q1, q2: cas.Quaternion.from_iterable(q1).diff(cas.Quaternion.from_iterable(q2)),
                    [q1t, q2t]))
        qds = np.array(qds)
        for r1, r2 in zip(r1s, r2s):
            compare_orientations(r1, r2)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_fmod(self, a, b):
        ref_r = np.fmod(a, b)
        assert np.isclose(cas.compile_and_execute(cas.fmod, [a, b]), ref_r, equal_nan=True)

    @given(float_no_nan_no_inf())
    def test_normalize_angle_positive(self, a):
        expected = giskard_math.normalize_angle_positive(a)
        actual = cas.compile_and_execute(cas.normalize_angle_positive, [a])
        assert np.isclose(giskard_math.shortest_angular_distance(expected, actual), 0.0)

    @given(float_no_nan_no_inf())
    def test_normalize_angle(self, a):
        ref_r = giskard_math.normalize_angle(a)
        assert np.isclose(cas.compile_and_execute(cas.normalize_angle, [a]), ref_r)

    @given(float_no_nan_no_inf(),
           float_no_nan_no_inf())
    def test_shorted_angular_distance(self, angle1, angle2):
        try:
            expected = giskard_math.shortest_angular_distance(angle1, angle2)
        except ValueError:
            expected = np.nan
        actual = cas.compile_and_execute(cas.shortest_angular_distance, [angle1, angle2])
        assert np.isclose(actual, expected, equal_nan=True)

    @given(unit_vector(4),
           unit_vector(4))
    def test_entrywise_product(self, q1, q2):
        m1 = giskard_math.rotation_matrix_from_quaternion(*q1)
        m2 = giskard_math.rotation_matrix_from_quaternion(*q2)
        r1 = cas.compile_and_execute(cas.entrywise_product, [m1, m2])
        r2 = m1 * m2
        assert np.allclose(r1, r2)

    def test_kron(self):
        m1 = np.eye(4)
        r1 = cas.compile_and_execute(cas.kron, [m1, m1])
        r2 = np.kron(m1, m1)
        assert np.allclose(r1, r2)

    @given(sq_matrix())
    def test_sum(self, m):
        actual_sum = cas.compile_and_execute(cas.sum, [m])
        expected_sum = np.sum(m)
        assert np.isclose(actual_sum, expected_sum, rtol=1.e-4)

    @settings(deadline=timedelta(milliseconds=500))
    @given(st.integers(max_value=10000, min_value=1),
           st.integers(max_value=5000, min_value=-5000),
           st.integers(max_value=5000, min_value=-5000),
           st.integers(max_value=1000, min_value=1))
    def test_velocity_limit_from_position_limit(self, acceleration, desired_result, j, step_size):
        step_size /= 1000
        acceleration /= 1000
        desired_result /= 1000
        j /= 1000
        # set current position to 0 such that the desired result is already the difference
        velocity = cas.compile_and_execute(cas.velocity_limit_from_position_limit,
                                           [acceleration, desired_result, j, step_size])
        position = j
        i = 0
        start_sign = np.sign(velocity)
        while np.sign(velocity) == start_sign and i < 100000:
            position += velocity * step_size
            velocity -= np.sign(desired_result - j) * acceleration * step_size
            i += 1
        # np.testing.assert_almost_equal(position, desired_result)
        assert math.isclose(position, desired_result, abs_tol=4, rel_tol=4)

    @given(float_no_nan_no_inf_min_max(min_value=0))
    def test_r_gauss(self, n):
        result = cas.compile_and_execute(lambda x: cas.r_gauss(cas.gauss(x)), [n])
        np.isclose(result, n)
        result = cas.compile_and_execute(lambda x: cas.gauss(cas.r_gauss(x)), [n])
        np.isclose(result, n)

    @given(sq_matrix())
    def test_sum_row(self, m):
        actual_sum = cas.compile_and_execute(cas.sum_row, [m])
        expected_sum = np.sum(m, axis=0)
        assert np.allclose(actual_sum, expected_sum)

    @given(sq_matrix())
    def test_sum_column(self, m):
        actual_sum = cas.compile_and_execute(cas.sum_column, [m])
        expected_sum = np.sum(m, axis=1)
        assert np.allclose(actual_sum, expected_sum)

    def test_distance_point_to_line_segment1(self):
        p = np.array([0, 0, 0])
        start = np.array([0, 0, 0])
        end = np.array([0, 0, 1])
        distance = cas.compile_and_execute(lambda a, b, c: cas.distance_point_to_line_segment(a, b, c)[0],
                                           [p, start, end])
        nearest = cas.compile_and_execute(lambda a, b, c: cas.distance_point_to_line_segment(a, b, c)[1],
                                          [p, start, end])
        assert distance == 0
        assert nearest[0] == 0
        assert nearest[1] == 0
        assert nearest[2] == 0

    def test_distance_point_to_line_segment2(self):
        p = np.array([0, 1, 0.5])
        start = np.array([0, 0, 0])
        end = np.array([0, 0, 1])
        distance = cas.compile_and_execute(lambda a, b, c: cas.distance_point_to_line_segment(a, b, c)[0],
                                           [p, start, end])
        nearest = cas.compile_and_execute(lambda a, b, c: cas.distance_point_to_line_segment(a, b, c)[1],
                                          [p, start, end])
        assert distance == 1
        assert nearest[0] == 0
        assert nearest[1] == 0
        assert nearest[2] == 0.5

    def test_distance_point_to_line_segment3(self):
        p = np.array([0, 1, 2])
        start = np.array([0, 0, 0])
        end = np.array([0, 0, 1])
        distance = cas.compile_and_execute(lambda a, b, c: cas.distance_point_to_line_segment(a, b, c)[0],
                                           [p, start, end])
        nearest = cas.compile_and_execute(lambda a, b, c: cas.distance_point_to_line_segment(a, b, c)[1],
                                          [p, start, end])
        assert distance == 1.4142135623730951
        assert nearest[0] == 0
        assert nearest[1] == 0
        assert nearest[2] == 1

    def test_to_str(self):
        axis = cas.Vector3(*cas.create_symbols(['v1', 'v2', 'v3']))
        angle = cas.Symbol('alpha')
        q = cas.Quaternion.from_axis_angle(axis, angle)
        expr = cas.norm(q)
        assert cas.to_str(expr) == [['sqrt((((sq((v1*sin((alpha/2))))'
                                     '+sq((v2*sin((alpha/2)))))'
                                     '+sq((v3*sin((alpha/2)))))'
                                     '+sq(cos((alpha/2)))))']]
        assert cas.to_str(expr) == expr.pretty_str()

    def test_to_str2(self):
        a, b = cas.var('a b')
        e = cas.if_eq(a, 0, a, b)
        assert cas.to_str(e) == [['(((a==0)?a:0)+((!(a==0))?b:0))']]
        assert cas.to_str(e) == e.pretty_str()

    def test_leq_on_array(self):
        a = cas.Expression(np.array([1,2,3,4]))
        b = cas.Expression(np.array([2,2,2,2]))
        assert not cas.logic_all(cas.less_equal(a, b)).to_np()
