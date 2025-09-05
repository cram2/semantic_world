from dataclasses import dataclass
from typing_extensions import Optional, Dict, Any, Self

from random_events.utils import SubclassJSONSerializer


@dataclass
class PrefixedName(SubclassJSONSerializer):
    name: str
    prefix: Optional[str] = None

    def __hash__(self):
        return hash((self.prefix, self.name))

    def __str__(self):
        return f"{self.prefix}/{self.name}"

    def __eq__(self, other):
        return str(self) == str(other)

    def to_json(self) -> Dict[str, Any]:
        return {**super().to_json(), "name": self.name, "prefix": self.prefix}

    @classmethod
    def _from_json(cls, data: Dict[str, Any]) -> Self:
        return cls(name=data["name"], prefix=data["prefix"])

    def __lt__(self, other):
        return str(self) < str(other)

    def __le__(self, other):
        return str(self) <= str(other)

    def __gt__(self, other):
        return str(self) > str(other)

    def __ge__(self, other):
        return str(self) >= str(other)
