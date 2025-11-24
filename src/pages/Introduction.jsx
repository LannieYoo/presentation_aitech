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
        <p>• Study ROS2 in the AISD program</p>
        <p>• Labs and assignment connect ROS2 with vision</p>
        <p>• First work with turtlesim and hand tracking</p>
        <p>• Later extend same idea to Create3 robot</p>
      </section>

      <section className="section">
        <h2>Goal of Our Project</h2>
        <p>• Use camera and MediaPipe hands to control turtlesim</p>
        <p>• Understand how ROS2 nodes, topics, publishers, and subscribers work together</p>
        <p>• Learn from real environment problems: wifi setup, camera support, network delay</p>
        <p>• Prepare useful tips for future students doing same labs</p>
      </section>

      <section className="section">
        <h2>System Overview</h2>
        <div className="flow-diagram">
          <div className="flow-item">ROS2 node reads camera frames</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">MediaPipe hands detects hand landmarks</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">ROS2 node sends motion commands to turtlesim</div>
          <div className="flow-arrow">↓</div>
          <div className="flow-item">Messages move over topics in ROS2</div>
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

