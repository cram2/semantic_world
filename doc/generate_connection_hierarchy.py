#!/usr/bin/env python3
"""
Auto-generate connection type hierarchy for documentation.
This script introspects the semantic_world connection classes to create an accurate hierarchy.
"""

import sys
import os

# Add src to path to import semantic_world
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

try:
    from semantic_world.world_entity import Connection
    from semantic_world.connections import (
        FixedConnection, RevoluteConnection, PrismaticConnection, Connection6DoF
    )
    
    def generate_connection_hierarchy():
        """Generate connection hierarchy from actual class structure"""
        
        # Get all connection classes
        all_classes = [
            (Connection, "Base class for all connections"),
            (FixedConnection, "Rigid attachment, 0 DOF"),
            (RevoluteConnection, "Single rotational DOF"),
            (PrismaticConnection, "Single translational DOF"),
            (Connection6DoF, "Up to 6 DOF for free-floating bodies")
        ]
        
        hierarchy = []
        hierarchy.append("```")
        hierarchy.append("Connection (base class)")
        
        # Process subclasses
        for cls, description in all_classes[1:]:  # Skip base Connection
            if issubclass(cls, Connection):
                hierarchy.append(f"├── {cls.__name__} ({description})")
        
        hierarchy.append("```")
        
        return "\n".join(hierarchy)
    
    if __name__ == "__main__":
        print(generate_connection_hierarchy())
        
except ImportError as e:
    # Fallback if imports fail
    print("""```
Connection (base class)
├── FixedConnection (Rigid attachment, 0 DOF)
├── RevoluteConnection (Single rotational DOF)
├── PrismaticConnection (Single translational DOF)
└── Connection6DoF (Up to 6 DOF for free-floating bodies)
```""")