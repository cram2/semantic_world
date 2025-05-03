from sqlalchemy import Column, Float, ForeignKey, Integer, MetaData, String, Table
from sqlalchemy.orm import registry, relationship
import semantic_world.geometry

metadata = MetaData()


t_Color = Table(
    'Color', metadata,
    Column('id', Integer, primary_key=True),
    Column('R', Float, nullable=False),
    Column('G', Float, nullable=False),
    Column('B', Float, nullable=False),
    Column('A', Float, nullable=False)
)

t_Scale = Table(
    'Scale', metadata,
    Column('id', Integer, primary_key=True),
    Column('x', Float, nullable=False),
    Column('y', Float, nullable=False),
    Column('z', Float, nullable=False)
)

t_Shape = Table(
    'Shape', metadata,
    Column('id', Integer, primary_key=True),
    Column('polymorphic_type', String)
)

t_Mesh = Table(
    'Mesh', metadata,
    Column('id', ForeignKey('Shape.id'), primary_key=True),
    Column('filename', String, nullable=False),
    Column('scale_id', ForeignKey('Scale.id'))
)

t_Primitive = Table(
    'Primitive', metadata,
    Column('id', ForeignKey('Shape.id'), primary_key=True),
    Column('color_id', ForeignKey('Color.id'))
)

t_Box = Table(
    'Box', metadata,
    Column('id', ForeignKey('Primitive.id'), primary_key=True),
    Column('scale_id', ForeignKey('Scale.id'))
)

t_Cylinder = Table(
    'Cylinder', metadata,
    Column('id', ForeignKey('Primitive.id'), primary_key=True),
    Column('width', Float, nullable=False),
    Column('height', Float, nullable=False)
)

t_Sphere = Table(
    'Sphere', metadata,
    Column('id', ForeignKey('Primitive.id'), primary_key=True),
    Column('radius', Float, nullable=False)
)

mapper_registry = registry(metadata=metadata)

m_Color = mapper_registry.map_imperatively(semantic_world.geometry.Color, t_Color, )

m_Scale = mapper_registry.map_imperatively(semantic_world.geometry.Scale, t_Scale, )

m_Shape = mapper_registry.map_imperatively(semantic_world.geometry.Shape, t_Shape, polymorphic_on = "polymorphic_type", polymorphic_identity = "Shape")

m_Mesh = mapper_registry.map_imperatively(semantic_world.geometry.Mesh, t_Mesh, properties = dict(scale=relationship("Scale", foreign_keys=[t_Mesh.c.scale_id])), polymorphic_identity = "Mesh", inherits = m_Shape)

m_Primitive = mapper_registry.map_imperatively(semantic_world.geometry.Primitive, t_Primitive, properties = dict(color=relationship("Color", foreign_keys=[t_Primitive.c.color_id])), polymorphic_identity = "Primitive", inherits = m_Shape)

m_Sphere = mapper_registry.map_imperatively(semantic_world.geometry.Sphere, t_Sphere, polymorphic_identity = "Sphere", inherits = m_Primitive)

m_Cylinder = mapper_registry.map_imperatively(semantic_world.geometry.Cylinder, t_Cylinder, polymorphic_identity = "Cylinder", inherits = m_Primitive)

m_Box = mapper_registry.map_imperatively(semantic_world.geometry.Box, t_Box, properties = dict(scale=relationship("Scale", foreign_keys=[t_Box.c.scale_id])), polymorphic_identity = "Box", inherits = m_Primitive)
