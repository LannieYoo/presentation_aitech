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
        <h2>iRobot Create3 Real Robot Demonstration</h2>
        <p>• Successfully tested ROS2 control with physical iRobot Create3</p>
        <p>• Demonstrated real-world robot navigation and control</p>
        
        <div className="video-container" style={{ margin: '2rem 0', width: '100%' }}>
          <video 
            controls 
            style={{ width: '100%', borderRadius: '8px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}
          >
            <source src={`${import.meta.env.BASE_URL}upfiles/create3_web.mp4`} type="video/mp4" />
            Your browser does not support the video tag.
          </video>
          <p style={{ 
            marginTop: '1rem', 
            fontSize: '0.9rem', 
            color: '#666',
            fontStyle: 'italic' 
          }}>
            iRobot Create3 demonstration with ROS2 control
          </p>
        </div>
      </section>

      <section className="section">
        <h2>WiFi Configuration Challenges with Create3</h2>
        <p><strong>Initial WiFi Setup Issues:</strong></p>
        <p>• Pre-configured WiFi settings on Create3 failed to connect properly</p>
        <p>• Robot could not establish stable connection with lab network</p>
        <p>• Had to troubleshoot network connectivity on the spot</p>
        
        <p style={{ marginTop: '1.5rem' }}><strong>Solution - Mobile Hotspot:</strong></p>
        <p>• Switched to personal mobile phone hotspot as alternative</p>
        <p>• Reconfigured Create3 WiFi settings to connect to phone</p>
        <p>• Successfully established stable connection for demonstration</p>
        <p>• This workaround allowed us to proceed with the demo</p>
        
        <p style={{ marginTop: '1.5rem' }}><strong>Key Challenge:</strong></p>
        <p>• WiFi configuration and network reliability proved to be the most difficult part</p>
        <p>• Requires physical access to robot and understanding of network settings</p>
        <p>• Emphasizes importance of testing network connectivity well before demo day</p>
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
            <p>Test WiFi connectivity with physical robots early - have backup network options ready</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 4</h3>
            <p>Mobile hotspot can be a reliable backup when lab network fails</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 5</h3>
            <p>Test camera support early, not just on demo day</p>
          </div>
          <div className="tip-card">
            <h3>💡 Tip 6</h3>
            <p>Do short dry run in lab before real demo</p>
          </div>
        </div>
      </section>
    </div>
  )
}

export default NetworkPerformance

