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
        <p>At first we tried to use a virtual machine on a personal laptop.</p>
        <p>Inside VMware the camera was not detected in a stable way.</p>
        <p>We spent a lot of time trying to make the camera work.</p>
        <p>Because the camera was not reliable it was hard to test MediaPipe.</p>
      </section>

      <section className="section">
        <h2>Switch to Loaner Laptop for Camera and Vision</h2>
        <p>We changed the plan and used the loaner laptop directly for vision.</p>
        <p>On the loaner laptop the camera worked much better.</p>
        <p>MediaPipe hands could detect the hand landmarks in real time.</p>
        <p>This change saved time and made the vision part possible.</p>
      </section>

      <section className="section">
        <h2>Move and Hands Python Files</h2>
        <div className="two-column">
          <div className="column">
            <h3>Move File</h3>
            <p>One python file sends move commands to turtlesim.</p>
            <p>It publishes linear and angular velocity to a turtlesim command topic.</p>
          </div>
          <div className="column">
            <h3>Hands File</h3>
            <p>Another python file reads camera frames from the camera.</p>
            <p>It uses MediaPipe hands to track the hand.</p>
            <p>It maps hand positions or gestures to motion commands.</p>
          </div>
        </div>
      </section>

      <section className="section">
        <h2>Full Flow for Hand Controlled Turtle</h2>
        <div className="flow-diagram">
          <div className="flow-item">Camera captures the hand</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">MediaPipe hands finds key points in the image</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">A ROS2 node converts the hand data to velocity messages</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">Turtlesim subscriber receives the messages and moves the turtle</div>
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

