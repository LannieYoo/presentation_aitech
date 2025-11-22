# ROS2 Vision Presentation Site

A modern, dark-themed presentation website for CST8504 Robot Programming with ROS2 course.

## Features

- 🎨 Modern dark theme with black background
- 📄 5-page presentation with smooth navigation
- 🔝 Top pagination navigation bar
- 📱 Fully responsive design
- ⚡ Built with React + Vite for fast performance
- 🚀 Deployed on GitHub Pages

## Pages

1. **Intro** - ROS2 Vision Project Overview
2. **ROS2 and Setup** - First steps with ROS2 and system setup
3. **Camera and Turtlesim** - Camera and hand control flow
4. **Network and Performance** - Network delays and lab environment issues
5. **Conclusion and Advice** - What we learned and advice for future students

## Getting Started

### Prerequisites

- Node.js (v16 or higher)
- npm or yarn

### Installation

```bash
# Install dependencies
npm install
```

### Development

```bash
# Start development server
npm run dev
```

The site will be available at `http://localhost:5173`

### Build

```bash
# Build for production
npm run build
```

### Preview Production Build

```bash
# Preview the production build
npm run preview
```

## GitHub Pages Deployment

This project is configured for automatic deployment to GitHub Pages.

1. Push to `master` or `main` branch
2. GitHub Actions will automatically build and deploy
3. Site will be available at: `https://lannieyoo.github.io/presentation_aitech/`

### Manual Deployment

If you need to deploy manually:

```bash
npm run build
# Then push the dist folder to gh-pages branch or use GitHub Actions
```

## Project Structure

```
src/
├── components/
│   ├── Pagination.jsx      # Top navigation pagination
│   └── Pagination.css
├── pages/
│   ├── Intro.jsx           # Page 1: Intro
│   ├── ROS2Setup.jsx      # Page 2: ROS2 and Setup
│   ├── CameraTurtlesim.jsx # Page 3: Camera and Turtlesim
│   ├── NetworkPerformance.jsx # Page 4: Network and Performance
│   ├── Conclusion.jsx      # Page 5: Conclusion
│   └── Page.css            # Shared page styles
├── App.jsx                 # Main app component with routing
├── App.css
├── main.jsx               # Entry point
└── index.css              # Global styles
```

## Team

- Jiaxing YI (041127158)
- Hye Ran Yoo (041145212)
- Joseph Weng (041076091)
- Peng Wang (041107730)

## Course

CST8504 Robot Programming with ROS2
