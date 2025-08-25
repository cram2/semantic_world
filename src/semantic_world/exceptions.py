from __future__ import annotations
from typing_extensions import List, TYPE_CHECKING

from .prefixed_name import PrefixedName

if TYPE_CHECKING:
    from .world_entity import View


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
        msg = f'View {view} already exists.'
        super().__init__(msg)


class DuplicateViewError(UsageError):
    def __init__(self, views: List[View]):
        msg = f'Views {views} are duplicates, while views elements should be unique.'
        super().__init__(msg)


class ViewNotFoundError(UsageError):
    def __init__(self, name: PrefixedName):
        msg = f'View with name {name} not found'
        super().__init__(msg)

class AlreadyBelongsToAWorldError(UsageError):
    ...
