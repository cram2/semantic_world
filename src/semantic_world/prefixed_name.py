from dataclasses import dataclass
from typing import Optional


@dataclass
class PrefixedName:
    name: str
    prefix: Optional[str] = None

    def __hash__(self):
        return hash((self.prefix, self.name))

    def __str__(self):
        return f"{self.prefix}/{self.name}"

    def __eq__(self, other):
        return str(self) == str(other)

    def __lt__(self, other):
        return str(self) < str(other)

    def __le__(self, other):
        return str(self) <= str(other)

    def __gt__(self, other):
        return str(self) > str(other)

    def __ge__(self, other):
        return str(self) >= str(other)
