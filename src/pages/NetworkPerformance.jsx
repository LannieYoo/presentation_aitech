import './Page.css'

function NetworkPerformance() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>Network Delays and Lab Environment Issues</h1>
        <p className="presenter">Speaker: Peng Wang</p>
      </div>

      <section className="section">
        <h2>Slow Camera and Turtle Motion Over the Network</h2>
        <p>In some tests we used the loaner laptop as the main ROS2 machine.</p>
        <p>We connected to it from Putty on a personal laptop.</p>
        <p>In this case camera frames and turtle motion felt slow.</p>
        <p>Hand movement and turtle reaction had clear delay.</p>
      </section>

      <section className="section">
        <h2>Lab Network Congestion</h2>
        <p>During lab time many students used the same network.</p>
        <p>Sometimes the ssh connection was unstable.</p>
        <p>This caused extra lag and even disconnects.</p>
        <p>It made the demo less smooth and harder to debug.</p>
      </section>

      <section className="section">
        <h2>Running Directly on the Loaner Laptop</h2>
        <p>When we ran the terminals directly on the loaner laptop the system was much faster.</p>
        <p>The turtle reaction to hand movement was more real time.</p>
        <p>The main downside was that we had to type long commands directly on that machine.</p>
        <p>Long ros2 launch and ros2 run commands were easy to mistype.</p>
      </section>

      <section className="section">
        <h2>Practical Tips for Future Students</h2>
        <div className="tips-grid">
          <div className="tip-card">
            <h3>💡 Tip 1</h3>
            <p>If the network is slow, try to run the main ROS2 commands directly on the machine with the camera.</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 2</h3>
            <p>Prepare a text file with the common ros2 commands so you can copy and paste.</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 3</h3>
            <p>Test camera support early, not just on the demo day.</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 4</h3>
            <p>Do a short dry run in the lab before the real demo.</p>
          </div>
        </div>
      </section>
    </div>
  )
}

export default NetworkPerformance

