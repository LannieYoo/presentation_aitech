import './Page.css'
import { Link } from 'react-router-dom'

function Intro() {
  return (
    <div className="home-page">
      <div className="home-hero">
        <div className="home-content">
          <h1 className="home-title">ROS2 Vision Project</h1>
          <p className="home-subtitle">Hand Control for Turtlesim</p>
          <p className="home-description">
            Using camera and MediaPipe hands to control turtlesim through ROS2
          </p>

          <div className="home-team">
            <h2 className="home-team-title">Team Members</h2>
            <div className="home-team-grid">
              <div className="home-team-card">
                <h3 className="team-card-name">Jiaxing YI</h3>
                <p className="team-card-id">041127158</p>
              </div>
              <div className="home-team-card">
                <h3 className="team-card-name">Hye Ran Yoo</h3>
                <p className="team-card-id">041145212</p>
              </div>
              <div className="home-team-card">
                <h3 className="team-card-name">Joseph Weng</h3>
                <p className="team-card-id">041076091</p>
              </div>
              <div className="home-team-card">
                <h3 className="team-card-name">Peng Wang</h3>
                <p className="team-card-id">041107730</p>
              </div>
            </div>
          </div>

          <div className="home-navigation">
            <Link to="/introduction" className="home-nav-button">
              Start Presentation →
            </Link>
          </div>
        </div>
      </div>
    </div>
  )
}

export default Intro

