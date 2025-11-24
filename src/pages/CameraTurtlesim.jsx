import './Page.css'

function CameraTurtlesim() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>Camera and Hand Control Flow</h1>
        <p className="presenter">Speaker: Joseph Weng</p>
      </div>

      <section className="section">
        <h2>Camera Problem on VMware</h2>
        <p>• Initially tried using a virtual machine on personal laptop</p>
        <p>• Camera detection was unstable inside VMware</p>
        <p>• Spent significant time troubleshooting camera issues</p>
        <p>• Unreliable camera made MediaPipe testing difficult</p>
      </section>

      <section className="section">
        <h2>Switch to Loaner Laptop for Camera and Vision</h2>
        <p>• Switched to loaner laptop for direct camera access</p>
        <p>• Camera performance significantly improved</p>
        <p>• MediaPipe hands successfully detected hand landmarks in real-time</p>
        <p>• Solution saved time and enabled vision implementation</p>
      </section>

      <section className="section">
        <h2>Move and Hands Python Files</h2>
        <div className="two-column">
          <div className="column">
            <h3>Move File</h3>
            <p>• Sends move commands to turtlesim</p>
            <p>• Publishes linear and angular velocity to command topic</p>
          </div>
          <div className="column">
            <h3>Hands File</h3>
            <p>• Reads camera frames from camera</p>
            <p>• Uses MediaPipe hands for hand tracking</p>
            <p>• Maps hand positions/gestures to motion commands</p>
          </div>
        </div>
      </section>

      <section className="section">
        <h2>Full Flow for Hand Controlled Turtle</h2>
        <div className="flow-diagram">
          <div className="flow-item">Camera captures hand</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">MediaPipe detects hand key points</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">ROS2 node converts to velocity messages</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">Turtlesim receives messages and moves turtle</div>
        </div>
      </section>

      <section className="section">
        <h2>Example Commands</h2>
        <div className="code-block">
          <code>ros2 run aisd_vision hands</code>
        </div>
        <div className="code-block">
          <code>ros2 run aisd_motion move</code>
        </div>
        <div className="code-block">
          <code>ros2 run turtlesim turtlesim_node</code>
        </div>
      </section>
    </div>
  )
}

export default CameraTurtlesim

