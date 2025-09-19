from __future__ import annotations

from typing import Iterable, Tuple, Union

from typing_extensions import Optional, List, Type, TYPE_CHECKING

from .datastructures.prefixed_name import PrefixedName

if TYPE_CHECKING:
    from .world import World
    from .world_description.world_entity import View, WorldEntity
    from .spatial_types.spatial_types import Symbol


class LogicalError(Exception):
    """
    An error that happens due to mistake in the logical operation or usage of the API during runtime.
    """

    ...


class UsageError(LogicalError):
    """
    An exception raised when an incorrect usage of the API is encountered.
    """

    ...


class AddingAnExistingViewError(UsageError):
    def __init__(self, view: View):
        msg = f"View {view} already exists."
        super().__init__(msg)


class DuplicateViewError(UsageError):
    def __init__(self, views: List[View]):
        msg = f"Views {views} are duplicates, while views elements should be unique."
        super().__init__(msg)


class DuplicateKinematicStructureEntityError(UsageError):
    def __init__(self, names: List[PrefixedName]):
        msg = f"Kinematic structure entities with names {names} are duplicates, while kinematic structure entity names should be unique."
        super().__init__(msg)


class SpatialTypesError(UsageError):
    pass


class WrongDimensionsError(SpatialTypesError):
    def __init__(
        self,
        expected_dimensions: Union[Tuple[int, int], str],
        actual_dimensions: Tuple[int, int],
    ):
        msg = f"Expected {expected_dimensions} dimensions, but got {actual_dimensions}."
        super().__init__(msg)


class NotSquareMatrixError(SpatialTypesError):
    def __init__(self, actual_dimensions: Tuple[int, int]):
        msg = f"Expected a square matrix, but got {actual_dimensions} dimensions."
        super().__init__(msg)


class HasFreeSymbolsError(SpatialTypesError):
    """
    Raised when an operation can't be performed on an expression with free symbols.
    """

    def __init__(self, symbols: Iterable[Symbol]):
        msg = f"Operation can't be performed on expression with free symbols: {list(symbols)}."
        super().__init__(msg)


class ParsingError(Exception):
    """
    An error that happens during parsing of files.
    """

    def __init__(self, file_path: Optional[str] = None, msg: Optional[str] = None):
        if not msg:
            if file_path:
                msg = f"File {file_path} could not be parsed."
            else:
                msg = ""
        super().__init__(msg)


class ViewNotFoundError(UsageError):
    def __init__(self, name: PrefixedName):
        msg = f"View with name {name} not found"
        super().__init__(msg)


class AlreadyBelongsToAWorldError(UsageError):
    def __init__(self, world: World, type_trying_to_add: Type[WorldEntity]):
        msg = f"Cannot add a {type_trying_to_add} that already belongs to another world {world.name}."
        super().__init__(msg)
