import './Page.css'

function CameraTurtlesim() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>Camera and Hand Control Flow</h1>
        <p className="presenter">Speaker: Joseph Weng</p>
      </div>

      <section className="section">
        <h2>Camera Issue & Solution</h2>
        <p>VMware camera detection was unstable → Switched to loaner laptop</p>
        <p>Loaner laptop camera worked perfectly with MediaPipe hands</p>
      </section>

      <section className="section">
        <h2>Implementation</h2>
        <div className="two-column">
          <div className="column">
            <h3>Hands File</h3>
            <p>Reads camera, uses MediaPipe to track hand landmarks</p>
          </div>
          <div className="column">
            <h3>Move File</h3>
            <p>Publishes velocity commands to turtlesim</p>
          </div>
        </div>
      </section>

      <section className="section">
        <h2>Control Flow</h2>
        <div className="flow-diagram">
          <div className="flow-item">Camera → MediaPipe Hands</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">ROS2 Node → Velocity Messages</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">Turtlesim Moves</div>
        </div>
      </section>

      <section className="section">
        <h2>Key Commands</h2>
        <div className="code-block">
          <code>ros2 run aisd_vision hands</code><br />
          <code>ros2 run aisd_motion move</code><br />
          <code>ros2 run turtlesim turtlesim_node</code>
        </div>
      </section>
    </div>
  )
}

export default CameraTurtlesim

