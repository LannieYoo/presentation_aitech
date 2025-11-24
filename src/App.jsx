import { useEffect } from 'react'
import { BrowserRouter as Router, Routes, Route, useLocation } from 'react-router-dom'
import Pagination from './components/Pagination'
import Intro from './pages/Intro'
import Introduction from './pages/Introduction'
import ROS2Setup from './pages/ROS2Setup'
import CameraTurtlesim from './pages/CameraTurtlesim'
import NetworkPerformance from './pages/NetworkPerformance'
import Conclusion from './pages/Conclusion'
import './App.css'

const pages = [
  { path: '/', title: 'Home', component: Intro },
  { path: '/introduction', title: 'Introduction', component: Introduction },
  { path: '/ros2-setup', title: 'ROS2 & Setup', component: ROS2Setup },
  { path: '/camera-turtlesim', title: 'Camera & Turtlesim', component: CameraTurtlesim },
  { path: '/network-performance', title: 'Network & Performance', component: NetworkPerformance },
  { path: '/conclusion', title: 'Conclusion & Advice', component: Conclusion },
]

function ScrollToTop() {
  const { pathname } = useLocation()

  useEffect(() => {
    window.scrollTo(0, 0)
  }, [pathname])

  return null
}

function App() {
  return (
    <Router basename="/presentation_aitech">
      <ScrollToTop />
      <div className="app">
        <Pagination pages={pages} />
        <main className="main-content">
          <Routes>
            {pages.map(({ path, component: Component }) => (
              <Route key={path} path={path} element={<Component />} />
            ))}
          </Routes>
        </main>
      </div>
    </Router>
  )
}

export default App

