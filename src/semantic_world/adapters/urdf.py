from ..world import World


class URDFAdapter:
    """
    Stump class for loading worlds from a Unified Robot Description Format (URDF) file.
    """

    def load(self, *args, **kwargs) -> World:
        """
        Create a new world from the URDF file.
        """
        ...