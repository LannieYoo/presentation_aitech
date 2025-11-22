import './Page.css'

function Conclusion() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>What We Learned and Advice for Future Students</h1>
        <p className="presenter">Speaker: Jiaxing YI</p>
      </div>

      <section className="section">
        <h2>Key Learnings</h2>
        <p>ROS2 concepts (node, topic, publisher, subscriber) became clear with practice.</p>
        <p>Biggest challenges: <strong>wifi setup, camera detection, network delay</strong> - not the code logic.</p>
        <p>Once environment worked, everything else was manageable.</p>
      </section>

      <section className="section">
        <h2>What Helped</h2>
        <ul className="key-points">
          <li>Fix wifi → Test ROS2 → Test camera → Add MediaPipe</li>
          <li>Clear mental model: Laptop → ROS2 nodes → Topics → Turtlesim</li>
          <li>Same pattern works for Create3 robot</li>
        </ul>
      </section>

      <section className="section">
        <h2>Advice for Future Students</h2>
        <div className="advice-grid">
          <div className="advice-card">
            <p>Don't panic - ROS2 gets easier with practice</p>
          </div>
          <div className="advice-card">
            <p>Draw diagrams before coding</p>
          </div>
          <div className="advice-card">
            <p>Test each part separately</p>
          </div>
          <div className="advice-card">
            <p>Avoid VMs for camera work</p>
          </div>
        </div>
      </section>

      <section className="section closing">
        <h2>Closing</h2>
        <p className="closing-text">We learned ROS2 by solving real environment problems.</p>
        <p className="closing-text highlight">We hope this makes your assignment smoother and less stressful.</p>
      </section>
    </div>
  )
}

export default Conclusion

