class WorldException(Exception):
    pass


class UnknownGroupException(WorldException, KeyError):
    pass


class UnknownLinkException(WorldException, KeyError):
    pass


class UnknownJointException(WorldException, KeyError):
    pass


class InvalidWorldOperationException(WorldException, KeyError):
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
