import './Page.css'

function NetworkPerformance() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>Network Delays and Lab Environment Issues</h1>
        <p className="presenter">Speaker: Peng Wang</p>
      </div>

      <section className="section">
        <h2>Network Performance Issues</h2>
        <p>SSH connection from personal laptop → Slow camera frames and turtle motion</p>
        <p>Lab network congestion → Unstable connections during peak hours</p>
        <p>Solution: Run commands directly on loaner laptop for real-time performance</p>
      </section>

      <section className="section">
        <h2>Key Tips</h2>
        <div className="tips-grid">
          <div className="tip-card">
            <h3>💡 Run locally</h3>
            <p>Execute ROS2 commands directly on the camera machine</p>
          </div>
          <div className="tip-card">
            <h3>💡 Prepare commands</h3>
            <p>Save common ros2 commands in a text file for copy-paste</p>
          </div>
          <div className="tip-card">
            <h3>💡 Test early</h3>
            <p>Test camera and network before demo day</p>
          </div>
        </div>
      </section>
    </div>
  )
}

export default NetworkPerformance

