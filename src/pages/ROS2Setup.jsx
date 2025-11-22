import './Page.css'

function ROS2Setup() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>First Steps with ROS2 and System Setup</h1>
        <p className="presenter">Speaker: Hye Ran Yoo</p>
      </div>

      <section className="section">
        <h2>ROS2 Core Concepts</h2>
        <div className="terms-grid">
          <div className="term-card">
            <h3>Node</h3>
            <p>Running program in ROS2</p>
          </div>
          <div className="term-card">
            <h3>Publisher</h3>
            <p>Sends messages</p>
          </div>
          <div className="term-card">
            <h3>Subscriber</h3>
            <p>Receives messages</p>
          </div>
          <div className="term-card">
            <h3>Topic</h3>
            <p>Channel connecting them</p>
          </div>
        </div>
      </section>

      <section className="section">
        <h2>Main Challenge: Wifi Setup</h2>
        <p>Loaner Linux server had no wifi connection.</p>
        <p>We had to fix wifi first before any ROS2 work.</p>
        <p>After setup, basic commands worked: <code>ros2 node list</code>, <code>ros2 topic list</code></p>
      </section>

      <section className="section">
        <h2>System Overview</h2>
        <div className="flow-diagram">
          <div className="flow-item">Camera</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">MediaPipe Hands</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">ROS2 Node → Topics</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">Turtlesim Control</div>
        </div>
      </section>

      <section className="section">
        <h2>Our Strategy</h2>
        <ul className="key-points">
          <li>Fix environment first, then code</li>
          <li>Keep it simple and working</li>
          <li>Document problems and solutions</li>
        </ul>
      </section>
    </div>
  )
}

export default ROS2Setup

