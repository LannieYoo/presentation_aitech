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
        <p>• ROS2 was a new concept for our team</p>
        <p>• ROS2 runs as nodes that communicate over topics</p>
        <p>• Laptops use SSH client (Putty) to connect to Linux server</p>
        <p>• Server runs ROS2 nodes controlling turtlesim and Create3</p>
      </section>

      <section className="section">
        <h2>Loaner Linux Server and Wifi Problem</h2>
        <p>• Received Linux loaner server for assignment</p>
        <p>• WiFi was not working on the machine</p>
        <p>• Difficult to install packages and run system updates</p>
        <p>• Spent time setting up WiFi before ROS2 work</p>
        <p>• After WiFi setup, could run basic ROS2 commands</p>

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
          <div className="arch-item">Laptop runs Putty or SSH client</div>
          <div className="arch-arrow">↓</div>
          <div className="arch-item">Putty connects to loaner Linux server</div>
          <div className="arch-arrow">↓</div>
          <div className="arch-item">Server runs ROS2 nodes for camera, MediaPipe hands, and turtlesim control</div>
          <div className="arch-arrow">↓</div>
          <div className="arch-item">Messages move over topics between nodes</div>
        </div>
      </section>
    </div>
  )
}

export default ROS2Setup

