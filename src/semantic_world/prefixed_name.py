from dataclasses import dataclass
from typing import Optional


@dataclass
class PrefixedName:
    name: str
    prefix: Optional[str] = "world"

    def __hash__(self):
        return hash((self.prefix, self.name))

    def __str__(self):
        return f"{self.prefix}/{self.name}"