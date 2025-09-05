import os
import mujoco as mj

from semantic_world.adapters.digital_twin_adapter import (
    build_cabinet_view, DigitalTwinCabinetAgent
)

def test_semantic_view_controls_digital_twin(tmp_path):
    # Minimal scene: cabinet body, door body with a hinge, and a handle body
    xml = """
    <mujoco>
      <worldbody>
        <body name="cabinet">
          <body name="cabinet_door">
            <joint name="cabinet_door_hinge" type="hinge" axis="0 0 1" limited="true" range="0 90"/>
            <geom type="box" size="0.05 0.3 0.4"/>
            <body name="cabinet_handle">
              <geom type="box" size="0.01 0.02 0.1"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    xml_path = os.path.join(tmp_path, "scene.xml")
    with open(xml_path, "w") as f:
        f.write(xml)

    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)

    # 1) Build the semantic view from the MuJoCo model
    cab = build_cabinet_view(
        model, data,
        cabinet_body="cabinet",
        door_body="cabinet_door",
        handle_body="cabinet_handle",
        door_joint="cabinet_door_hinge",
    )

    # 2) Bind a tiny 'agent' that uses the semantic view to control the simulator
    agent = DigitalTwinCabinetAgent(model=model, data=data, cab=cab)

    # 3) Close, then open to 60%, then assert it's open
    agent.close()
    assert not agent.is_open(5.0)

    agent.open(60.0)
    assert agent.is_open(20.0)  # should register as open beyond 20% threshold

    # 4) Close again
    agent.close()
    assert not agent.is_open(5.0)

