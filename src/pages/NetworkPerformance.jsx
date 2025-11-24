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
        <p>• Used loaner laptop as main ROS2 machine</p>
        <p>• Connected via Putty from personal laptop</p>
        <p>• Camera frames and turtle motion felt slow</p>
        <p>• Clear delay between hand movement and turtle reaction</p>
      </section>

      <section className="section">
        <h2>Lab Network Congestion</h2>
        <p>• Many students used same network during lab time</p>
        <p>• SSH connection was sometimes unstable</p>
        <p>• Caused extra lag and disconnects</p>
        <p>• Made demo less smooth and harder to debug</p>
      </section>

      <section className="section">
        <h2>Running Directly on the Loaner Laptop</h2>
        <p>• Running terminals directly on loaner laptop was much faster</p>
        <p>• Turtle reaction to hand movement was more real-time</p>
        <p>• Had to type long commands directly on that machine</p>
        <p>• Long ros2 launch/run commands were easy to mistype</p>
      </section>

      <section className="section">
        <h2>Practical Tips for Future Students</h2>
        <div className="tips-grid">
          <div className="tip-card">
            <h3>💡 Tip 1</h3>
            <p>Run main ROS2 commands directly on machine with camera if network is slow</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 2</h3>
            <p>Prepare text file with common ros2 commands for copy/paste</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 3</h3>
            <p>Test camera support early, not just on demo day</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 4</h3>
            <p>Do short dry run in lab before real demo</p>
          </div>
        </div>
      </section>
    </div>
  )
}

export default NetworkPerformance

