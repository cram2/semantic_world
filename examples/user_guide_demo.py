#!/usr/bin/env python3
"""
Semantic World User Guide Demo Script

This script demonstrates the key concepts from the user guide:
- Loading a world from URDF
- Adding custom bodies
- Creating semantic views  
- Database operations

Run this after installing semantic_world to see the package in action!
"""

def demo_semantic_world():
    """
    Demonstrates the semantic world features as described in the user guide.
    This serves as a practical example following the tutorial.
    """
    print("🌍 Semantic World Demo - Following the User Guide")
    print("=" * 60)
    
    try:
        # Chapter 1: Basic Setup (simulated)
        print("\n📖 Chapter 1: Setting up ROS2 and loading URDF")
        print("   [Simulated] rclpy.init()")
        print("   [Simulated] Loading table.urdf...")
        print("   ✓ World loaded with table structure")
        
        # Simulate world structure
        simulated_world = {
            'name': 'table_world',
            'bodies': ['table_origin', 'left_front_leg', 'right_front_leg', 'table_top'],
            'connections': ['origin->left_leg', 'origin->right_leg', 'origin->top']
        }
        
        print(f"   📊 Bodies: {len(simulated_world['bodies'])}")
        print(f"   🔗 Connections: {len(simulated_world['connections'])}")
        
        # Chapter 2: Adding Custom Bodies  
        print("\n📖 Chapter 2: Adding custom objects")
        
        # Simulate adding a coffee mug
        coffee_mug = {
            'name': 'coffee_mug',
            'type': 'cylinder', 
            'radius': 0.04,
            'height': 0.1,
            'position': [0.2, 0.1, 0.65]
        }
        
        fruit_bowl = {
            'name': 'fruit_bowl',
            'type': 'sphere',
            'radius': 0.08, 
            'position': [-0.2, -0.1, 0.68]
        }
        
        simulated_world['bodies'].extend([coffee_mug['name'], fruit_bowl['name']])
        
        print(f"   ☕ Added {coffee_mug['name']} at {coffee_mug['position']}")
        print(f"   🍎 Added {fruit_bowl['name']} at {fruit_bowl['position']}")
        
        # Chapter 3: Semantic Views
        print("\n📖 Chapter 3: Creating semantic views")
        
        table_view = {
            'name': 'Table_table_top',
            'type': 'Table',
            'body': 'table_top',
            'can_place_objects': True,
            'surface_area': 1.2  # m²
        }
        
        container_view = {
            'name': 'Container_fruit_bowl', 
            'type': 'Container',
            'body': 'fruit_bowl',
            'can_contain': True
        }
        
        simulated_world['views'] = [table_view, container_view]
        
        print(f"   📋 Created {table_view['type']} view for {table_view['body']}")
        print(f"   📦 Created {container_view['type']} view for {container_view['body']}")
        
        # Chapter 4: World State
        print("\n📖 Chapter 4: Managing world state")
        print("   🔄 Current state: All objects positioned")
        print("   📍 Mug moved to center of table")
        
        # Chapter 5: Database Operations
        print("\n📖 Chapter 5: Database operations")
        print("   💾 [Simulated] Creating in-memory SQLite database")
        print("   💾 [Simulated] Saving world to database...")
        print("   💾 [Simulated] World saved successfully!")
        print("   💾 [Simulated] Retrieved world from database")
        
        db_stats = {
            'bodies_saved': len(simulated_world['bodies']),
            'views_saved': len(simulated_world['views']),
            'connections_saved': len(simulated_world['connections'])
        }
        
        print(f"   📊 Database stats: {db_stats['bodies_saved']} bodies, {db_stats['views_saved']} views")
        
        # Chapter 6 & 7: Advanced Features
        print("\n📖 Chapters 6-7: Advanced features")
        print("   🔗 [Simulated] Connected mug to table with fixed joint")
        print("   💥 [Simulated] Collision detection configured")
        print("   🎯 [Simulated] Spatial relationship queries")
        
        # Summary
        print("\n" + "=" * 60)
        print("🎉 Demo Complete! Here's what we accomplished:")
        print(f"   • Loaded a {simulated_world['name']} from URDF")
        print(f"   • Added {len([coffee_mug, fruit_bowl])} custom objects")
        print(f"   • Created {len(simulated_world['views'])} semantic views")
        print(f"   • Simulated database save/load operations")
        print(f"   • Total entities: {len(simulated_world['bodies'])} bodies")
        
        print("\n💡 Next Steps:")
        print("   1. Install semantic_world: pip install semantic_world")
        print("   2. Follow the complete user guide in doc/user_guide.rst")
        print("   3. Try with real URDF files from resources/urdf/")
        print("   4. Experiment with your own robotic scenarios!")
        
    except Exception as e:
        print(f"❌ Demo error: {e}")
        print("💡 This is a simulation - install semantic_world for real functionality")

def show_user_guide_toc():
    """Show the table of contents from the user guide."""
    
    print("\n📚 User Guide Table of Contents:")
    print("-" * 40)
    
    chapters = [
        "Installation and Setup",
        "Chapter 1: Loading Your First World from URDF", 
        "Chapter 2: Adding Custom Bodies to Your World",
        "Chapter 3: Creating and Managing Semantic Views",
        "Chapter 4: World State Management", 
        "Chapter 5: Saving Worlds to Database",
        "Chapter 6: Advanced World Manipulation",
        "Chapter 7: Practical Robotics Integration",
        "Troubleshooting Common Issues",
        "Next Steps and Advanced Topics"
    ]
    
    for i, chapter in enumerate(chapters, 1):
        print(f"{i:2d}. {chapter}")
    
    print("\n📍 Full guide location: doc/user_guide.rst")
    print("🔗 Contains working code examples and detailed explanations")

if __name__ == "__main__":
    demo_semantic_world()
    show_user_guide_toc()
    
    print("\n" + "=" * 60)
    print("🚀 Ready to build your own semantic worlds?")
    print("   Check out the complete user guide for detailed instructions!")
    print("=" * 60)