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
        <h2>First Contact with ROS2</h2>
        <p>ROS2 was a new concept for our team.</p>
        <p>We needed to understand that ROS2 runs as nodes that talk over topics.</p>
        <p>Our laptops use an ssh client such as Putty to connect to a Linux server.</p>
        <p>The server runs the ROS2 nodes that control turtlesim, and later can control Create3.</p>
      </section>

      <section className="section">
        <h2>Loaner Linux Server and Wifi Problem</h2>
        <p>At the start we received a Linux loaner server for the assignment.</p>
        <p>Wifi was not working on that machine.</p>
        <p>Because of that it was hard to install packages and run system updates.</p>
        <p>We spent time to set up wifi first before we could do ROS2 work.</p>
        <p>After wifi setup we could finally run basic ROS2 commands.</p>

        <div className="setup-steps">
          <h3>Example Setup Steps</h3>
          <ol>
            <li>Get the loaner server from the lab.</li>
            <li>Test wifi and see that there is no connection.</li>
            <li>Configure wifi and confirm internet access.</li>
            <li>Run simple ros2 commands such as:
              <ul>
                <li><code>ros2 node list</code></li>
                <li><code>ros2 topic list</code></li>
              </ul>
            </li>
          </ol>
        </div>
      </section>

      <section className="section">
        <h2>Our Strategy for This Assignment</h2>
        <ul className="key-points">
          <li>Focus on getting a stable environment before writing complex code.</li>
          <li>Use a simple but working hand controlled turtlesim as our main demo.</li>
          <li>Document each problem and solution so that future students can follow the same steps.</li>
          <li>Keep the code small and clear instead of adding many features.</li>
        </ul>
      </section>

      <section className="section">
        <h2>Understanding the ROS2 Structure</h2>
        <div className="architecture-diagram">
          <div className="arch-item">Laptop runs Putty or another ssh client</div>
          <div className="arch-arrow">↓</div>
          <div className="arch-item">Putty connects to the loaner Linux server</div>
          <div className="arch-arrow">↓</div>
          <div className="arch-item">The loaner server runs ROS2 nodes for camera, MediaPipe hands, and turtlesim control</div>
          <div className="arch-arrow">↓</div>
          <div className="arch-item">Messages move over topics between these nodes</div>
        </div>
      </section>
    </div>
  )
}

export default ROS2Setup

