class WorldException(Exception):
    pass


class CorruptShapeException(WorldException):
    pass


class CorruptMeshException(CorruptShapeException):
    pass


class CorruptURDFException(CorruptShapeException):
    pass


class TransformException(WorldException):
    pass


class DuplicateNameException(WorldException):
    pass
