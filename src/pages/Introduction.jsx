import './Page.css'

function Introduction() {
  return (
    <div className="page">
      <div className="page-header">
        <h1>ROS2 Vision Project Overview</h1>
        <p className="presenter">Speaker: Jiaxing YI</p>
      </div>

      <section className="section">
        <h2>Context of the Course</h2>
        <p>We study ROS2 in the AISD program.</p>
        <p>The labs and the assignment ask us to connect ROS2 with vision.</p>
        <p>First we work with turtlesim and hand tracking.</p>
        <p>Later students can extend the same idea to the Create3 robot.</p>
      </section>

      <section className="section">
        <h2>Goal of Our Project</h2>
        <ul className="key-points">
          <li>Use a camera and MediaPipe hands to control turtlesim.</li>
          <li>Understand how ROS2 nodes, topics, publishers, and subscribers work together.</li>
          <li>Learn from real environment problems such as wifi setup, camera support, and network delay.</li>
          <li>Prepare useful tips for future students who will do the same labs.</li>
        </ul>
      </section>

      <section className="section">
        <h2>System Overview</h2>
        <div className="flow-diagram">
          <div className="flow-item">A ROS2 node reads camera frames</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">MediaPipe hands detects hand landmarks</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">Another ROS2 node sends motion commands to turtlesim</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">All messages move over topics inside ROS2</div>
        </div>
      </section>

      <section className="section">
        <h2>Roadmap for the Next Pages</h2>
        <div className="roadmap">
          <div className="roadmap-item">
            <span className="roadmap-number">1</span>
            <span>ROS2 and basic setup problems</span>
          </div>
          <div className="roadmap-item">
            <span className="roadmap-number">2</span>
            <span>Camera and hand control flow</span>
          </div>
          <div className="roadmap-item">
            <span className="roadmap-number">3</span>
            <span>Network and performance issues in the lab</span>
          </div>
          <div className="roadmap-item">
            <span className="roadmap-number">4</span>
            <span>What we learned and advice for AISD classmates</span>
          </div>
        </div>
      </section>
    </div>
  )
}

export default Introduction

