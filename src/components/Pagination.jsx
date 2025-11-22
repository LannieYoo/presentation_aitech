import { useState } from 'react'
import { useLocation, Link } from 'react-router-dom'
import './Pagination.css'

function Pagination({ pages }) {
  const location = useLocation()
  const currentIndex = pages.findIndex(page => page.path === location.pathname)
  const isHomePage = location.pathname === '/'
  const [isMenuOpen, setIsMenuOpen] = useState(false)
  
  const prevPage = currentIndex > 0 ? pages[currentIndex - 1] : null
  const nextPage = currentIndex < pages.length - 1 ? pages[currentIndex + 1] : null

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen)
  }

  const closeMenu = () => {
    setIsMenuOpen(false)
  }

  return (
    <nav className={`pagination ${isHomePage ? 'pagination-home' : ''}`}>
      <div className="pagination-container">
        <button 
          className="hamburger-menu"
          onClick={toggleMenu}
          aria-label="Toggle menu"
        >
          <span></span>
          <span></span>
          <span></span>
        </button>
        
        <div className={`pagination-links ${isMenuOpen ? 'menu-open' : ''}`}>
          {pages.map((page) => {
            const isActive = location.pathname === page.path
            return (
              <Link
                key={page.path}
                to={page.path}
                className={`pagination-link ${isActive ? 'active' : ''}`}
                onClick={closeMenu}
              >
                <span className="page-title">{page.title}</span>
              </Link>
            )
          })}
        </div>
        
        <div className="pagination-controls">
          <div className="pagination-info">
            <span>{currentIndex + 1} / {pages.length}</span>
          </div>
          <div className="pagination-buttons">
            {prevPage ? (
              <Link to={prevPage.path} className="pagination-button pagination-button-prev" onClick={closeMenu}>
                ← Prev
              </Link>
            ) : (
              <span className="pagination-button pagination-button-disabled">← Prev</span>
            )}
            {nextPage ? (
              <Link to={nextPage.path} className="pagination-button pagination-button-next" onClick={closeMenu}>
                Next →
              </Link>
            ) : (
              <span className="pagination-button pagination-button-disabled">Next →</span>
            )}
          </div>
        </div>
      </div>
    </nav>
  )
}

export default Pagination

