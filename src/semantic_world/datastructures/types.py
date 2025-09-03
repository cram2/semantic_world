from typing_extensions import Annotated, Literal
from typing_extensions import Sequence
import numpy as np
import numpy.typing as npt

AnyMatrix4x4 = Annotated[Sequence[Sequence[float]], Literal[4, 4]]
NpMatrix4x4 = Annotated[npt.NDArray[np.float64], Literal[4, 4]]
