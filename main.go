package main

import (
	"fmt"
	"image/color"
	"log"
	"math"
	"math/rand/v2"
	"runtime"
	"sync"
	"time"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

const (
	screenWidth   = 800
	screenHeight  = 600
	ballRadius    = 3
	maxSpeedColor = 50.0     // Adjust this value based on typical particle speeds
	fixedDt       = 1.0 / 60 // More stable with higher update rate
	cellSize      = ballRadius * 2
	cellsPerRow   = screenHeight / cellSize
	cellsPerCol   = screenWidth / cellSize
)

type ParticleSOA struct {
	x  []float64
	y  []float64
	vx []float64
	vy []float64
}

type ParticleGrid struct {
	grid        [cellsPerRow][cellsPerCol][]uint32
	density     [cellsPerRow][cellsPerCol]float64
	tempDensity [cellsPerRow][cellsPerCol]float64
}

func SetParticleGrid(p *ParticleSOA, g *ParticleGrid) {
	for i := 0; i < len(p.x); i++ {
		cellX := int(p.x[i] / cellSize)
		cellY := int(p.y[i] / cellSize)
		if cellX >= 0 && cellX < cellsPerCol && cellY >= 0 && cellY < cellsPerRow {
			g.grid[cellY][cellX] = append(g.grid[cellY][cellX], uint32(i))
		}
	}
}
func (g *ParticleGrid) ClearGrid() {
	for i := 0; i < cellsPerRow; i++ {
		for j := 0; j < cellsPerCol; j++ {
			g.grid[i][j] = nil
		}
	}
}

func (g *ParticleGrid) GetNeighbors(p *ParticleSOA, x, y float64) []uint32 {
	cellX := int(x / cellSize)
	cellY := int(y / cellSize)
	if cellX < 0 || cellX >= cellsPerCol || cellY < 0 || cellY >= cellsPerRow {
		fmt.Println("Particle out of bounds")
		return nil
	}
	neighbors := make([]uint32, 0)
	// Check the 8 neighboring cells
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			nCellX := cellX + dx
			nCellY := cellY + dy
			if nCellX >= 0 && nCellX < cellsPerCol && nCellY >= 0 && nCellY < cellsPerRow {
				neighbors = append(neighbors, g.grid[nCellY][nCellX]...)
			}
		}
	}
	// Remove duplicates
	uniqueNeighbors := make(map[uint32]struct{})
	for _, n := range neighbors {
		uniqueNeighbors[n] = struct{}{}
	}
	neighbors = make([]uint32, 0, len(uniqueNeighbors))
	for n := range uniqueNeighbors {
		neighbors = append(neighbors, n)
	}
	return neighbors
}

func (g *ParticleGrid) CalculateDensity() {
	for i := 0; i < cellsPerRow; i++ {
		for j := 0; j < cellsPerCol; j++ {
			// Calculate density as the number of particles in each cell
			g.density[i][j] = float64(len(g.grid[i][j]))
		}
	}
}

func (g *ParticleGrid) GaussianBlurDensity() {
	// Define a 3x3 Gaussian kernel
	kernel := [3][3]float64{
		{1.0 / 16.0, 2.0 / 16.0, 1.0 / 16.0},
		{2.0 / 16.0, 4.0 / 16.0, 2.0 / 16.0},
		{1.0 / 16.0, 2.0 / 16.0, 1.0 / 16.0},
	}

	// Apply the Gaussian kernel to each cell
	for i := 0; i < cellsPerRow; i++ {
		for j := 0; j < cellsPerCol; j++ {
			sum := 0.0
			for ki := -1; ki <= 1; ki++ {
				for kj := -1; kj <= 1; kj++ {
					ni := i + ki
					nj := j + kj
					if ni >= 0 && ni < cellsPerRow && nj >= 0 && nj < cellsPerCol {
						sum += g.density[ni][nj] * kernel[ki+1][kj+1]
					}
				}
			}
			g.tempDensity[i][j] = sum
		}
	}

	// Copy the blurred values back to the original density grid
	for i := 0; i < cellsPerRow; i++ {
		for j := 0; j < cellsPerCol; j++ {
			g.density[i][j] = g.tempDensity[i][j]
			g.tempDensity[i][j] = 0 // Reset tempDensity for next use
		}
	}
}

func (g *ParticleGrid) GetDensity(x, y float64) float64 {
	cellX := int(x / cellSize)
	cellY := int(y / cellSize)
	if cellX < 0 || cellX >= cellsPerCol || cellY < 0 || cellY >= cellsPerRow {
		return 0
	}
	return g.density[cellY][cellX]
}

func (g *ParticleGrid) GetDensityOfTheNeighbors(p *ParticleSOA, x, y float64) (densities []float64, positions [][2]float64, currentCellDensity float64, pX, pY int) {
	cellX := int(x / cellSize)
	cellY := int(y / cellSize)
	if cellX < 0 || cellX >= cellsPerCol || cellY < 0 || cellY >= cellsPerRow {
		return nil, nil, 0, 0, 0
	}
	// Check the 8 neighboring cells
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			nCellX := cellX + dx
			nCellY := cellY + dy
			if nCellX >= 0 && nCellX < cellsPerCol && nCellY >= 0 && nCellY < cellsPerRow {
				densities = append(densities, g.density[nCellY][nCellX])
				positions = append(positions, [2]float64{float64(nCellX) * cellSize, float64(nCellY) * cellSize})
			}
		}
	}
	return densities, positions, g.density[cellY][cellX], cellX * cellSize, cellY * cellSize
}

func (p *ParticleSOA) ApplyGravity(dt float64) {
	gravityEffect := 9.81 * dt * 10 // Gravity constant scaled by dt
	for i := 0; i < len(p.vy); i++ {
		p.vy[i] += gravityEffect
	}
}

// Improve damping for stability
func (p *ParticleSOA) ApplyDamping() {
	damping := 0.96 // Stronger damping for stability
	for i := 0; i < len(p.vx); i++ {
		p.vx[i] *= damping
		p.vy[i] *= damping
	}
}

// Fix collision resolution to avoid energy gain
func (p *ParticleSOA) ResolveCollisions(g *ParticleGrid) {
	// Use only a single iteration for stability
	for i := 0; i < len(p.x); i++ {
		cellX := int(p.x[i] / cellSize)
		cellY := int(p.y[i] / cellSize)

		// Skip if this particle is outside the grid
		if cellX < 0 || cellX >= cellsPerCol || cellY < 0 || cellY >= cellsPerRow {
			continue
		}
		
		// Check potential collisions with particles in neighboring cells
		for dy := -1; dy <= 1; dy++ {
			for dx := -1; dx <= 1; dx++ {
				nCellX := cellX + dx
				nCellY := cellY + dy

				// Skip if neighboring cell is outside the grid
				if nCellX < 0 || nCellX >= cellsPerCol || nCellY < 0 || nCellY >= cellsPerRow {
					continue
				}

				// Check all particles in this neighboring cell
				for _, j := range g.grid[nCellY][nCellX] {
					jInt := int(j)
					// Skip self
					if i == jInt || i > jInt { // Process each pair only once
						continue
					}

					// Calculate displacement and distance between particles
					dx := p.x[jInt] - p.x[i]
					dy := p.y[jInt] - p.y[i]
					distSq := dx*dx + dy*dy

					const epsilon = 0.0001
					minDist := 2.0*ballRadius + epsilon

					if distSq < minDist*minDist {
						if distSq < 1e-10 { // Almost perfect overlap, move slightly apart
							// Add small random jitter if particles are almost perfectly overlapping
							p.x[i] += (rand.Float64()*2 - 1.0) * ballRadius * 0.1
							p.y[i] += (rand.Float64()*2 - 1.0) * ballRadius * 0.1
							continue
						}

						// Calculate collision normal
						dist := math.Sqrt(distSq)
						nx := dx / dist
						ny := dy / dist

						// Exact correction (not over-correcting)
						overlap := minDist - dist
						correctionFactor := 1.0 // No over-correction to conserve energy

						// Apply position correction to separate particles
						moveX := nx * overlap * correctionFactor
						moveY := ny * overlap * correctionFactor

						p.x[i] -= moveX * 0.5
						p.y[i] -= moveY * 0.5
						p.x[jInt] += moveX * 0.5
						p.y[jInt] += moveY * 0.5

						// Calculate relative velocity
						rvx := p.vx[i] - p.vx[jInt]
						rvy := p.vy[i] - p.vy[jInt]

						// Calculate velocity along the normal
						velAlongNormal := rvx*nx + rvy*ny

						// Only apply impulse if particles are moving toward each other
						if velAlongNormal < 0 {
							// Lower restitution for more energy loss
							restitution := 0.3 // Lower restitution to dissipate energy
							j := -(1.0 + restitution) * velAlongNormal

							// Apply impulse
							impulseX := j * nx * 0.5
							impulseY := j * ny * 0.5

							p.vx[i] -= impulseX
							p.vy[i] -= impulseY
							p.vx[jInt] += impulseX
							p.vy[jInt] += impulseY
						}
					}
				}
			}
		}
	}
}

// ResolveBoundaries handles particle-boundary collisions with energy conservation.
func (p *ParticleSOA) ResolveBoundaries() {
	restitution := 0.7
	const epsilon = 0.0001
	for i := 0; i < len(p.x); i++ {
		// Left boundary
		if p.x[i] < ballRadius {
			// Only reverse velocity if moving toward boundary
			if p.vx[i] < 0 {
				p.vx[i] = -p.vx[i] * restitution
			}
			p.x[i] = ballRadius + epsilon
		} else if p.x[i] > screenWidth-ballRadius {
			// Only reverse velocity if moving toward boundary
			if p.vx[i] > 0 {
				p.vx[i] = -p.vx[i] * restitution
			}
			p.x[i] = screenWidth - ballRadius - epsilon
		}

		// Top boundary
		if p.y[i] < ballRadius {
			// Only reverse velocity if moving toward boundary
			if p.vy[i] < 0 {
				p.vy[i] = -p.vy[i] * restitution
			}
			p.y[i] = ballRadius + epsilon
		} else if p.y[i] > screenHeight-ballRadius {
			// Only reverse velocity if moving toward boundary
			if p.vy[i] > 0 {
				p.vy[i] = -p.vy[i] * restitution
			}
			p.y[i] = screenHeight - ballRadius - epsilon
		}
	}
}

var times map[string]time.Duration

func init() {
	times = make(map[string]time.Duration)
	times["ApplyDamping"] = 0
	times["ResolveCollisions"] = 0
	times["ResolveBoundaries"] = 0
	times["ClearGrid"] = 0
	times["SetParticleGrid"] = 0
	times["CalculateDensity"] = 0
	times["GaussianBlurDensity"] = 0
	times["ApplyDensityForces"] = 0
}

func (p *ParticleSOA) UpdateGrid(g *ParticleGrid) {
	g.ClearGrid()
	for i := 0; i < len(p.x); i++ {
		cellX := int(p.x[i] / cellSize)
		cellY := int(p.y[i] / cellSize)
		if cellX >= 0 && cellX < cellsPerCol && cellY >= 0 && cellY < cellsPerRow {
			g.grid[cellY][cellX] = append(g.grid[cellY][cellX], uint32(i))
		}
	}
}

// Streamline update function to avoid energy gain
func (p *ParticleSOA) Update(dt float64, g *ParticleGrid, gravity bool) {
	if gravity {
		p.ApplyGravity(dt)
	}

	// Store previous positions for velocity calculation
	prevX := make([]float64, len(p.x))
	prevY := make([]float64, len(p.y))
	for i := 0; i < len(p.x); i++ {
		prevX[i] = p.x[i]
		prevY[i] = p.y[i]
	}

	// Apply damping first
	start := time.Now()
	p.ApplyDamping()
	times["ApplyDamping"] = time.Duration(float64(times["ApplyDamping"])*0.9 + float64(time.Since(start))*0.1)

	// Update positions based on current velocities
	for i := 0; i < len(p.x); i++ {
		p.x[i] += p.vx[i] * dt
		p.y[i] += p.vy[i] * dt
	}

	// Update density grid
	start = time.Now()
	p.UpdateGrid(g) // Use the dedicated UpdateGrid method
	times["SetParticleGrid"] = time.Duration(float64(times["SetParticleGrid"])*0.9 + float64(time.Since(start))*0.1)

	start = time.Now()
	g.CalculateDensity()
	times["CalculateDensity"] = time.Duration(float64(times["CalculateDensity"])*0.9 + float64(time.Since(start))*0.1)

	start = time.Now()
	g.GaussianBlurDensity()
	times["GaussianBlurDensity"] = time.Duration(float64(times["GaussianBlurDensity"])*0.9 + float64(time.Since(start))*0.1)

	// Resolve collisions using grid-based approach
	start = time.Now()
	p.ResolveCollisions(g)
	times["ResolveCollisions"] = time.Duration(float64(times["ResolveCollisions"])*0.9 + float64(time.Since(start))*0.1)

	// Handle boundary collisions
	start = time.Now()
	p.ResolveBoundaries()
	times["ResolveBoundaries"] = time.Duration(float64(times["ResolveBoundaries"])*0.9 + float64(time.Since(start))*0.1)

	// Calculate new velocities based on position changes
	for i := 0; i < len(p.x); i++ {
		p.vx[i] = (p.x[i] - prevX[i]) / dt
		p.vy[i] = (p.y[i] - prevY[i]) / dt
	}

	// // Apply more damping at the end
	p.ApplyDamping()

	// Update the grid for density forces
	p.UpdateGrid(g)

	// Apply density forces
	start = time.Now()
	p.OptimizedApplyDensityForces(g)
	times["ApplyDensityForces"] = time.Duration(float64(times["ApplyDensityForces"])*0.9 + float64(time.Since(start))*0.1)
}

func (p *ParticleSOA) OptimizedApplyDensityForces(g *ParticleGrid) {
	numWorkers := runtime.NumCPU()
	particlesPerWorker := len(p.x) / numWorkers
	if particlesPerWorker < 1 {
		particlesPerWorker = 1
	}

	wg := sync.WaitGroup{}
	wg.Add(numWorkers)

	// Create worker pool
	for workerID := 0; workerID < numWorkers; workerID++ {
		startIdx := workerID * particlesPerWorker
		endIdx := (workerID + 1) * particlesPerWorker
		if workerID == numWorkers-1 {
			endIdx = len(p.x) // Make sure the last worker processes all remaining particles
		}

		go func(startIdx, endIdx int) {
			defer wg.Done()

			// Process particles in this worker's range
			for i := startIdx; i < endIdx; i++ {
				cellX := int(p.x[i] / cellSize)
				cellY := int(p.y[i] / cellSize)

				// Skip if outside grid
				if cellX < 0 || cellX >= cellsPerCol || cellY < 0 || cellY >= cellsPerRow {
					continue
				}

				currentDensity := g.density[cellY][cellX]
				netForceX := 0.0
				netForceY := 0.0

				// Check each neighboring cell
				for dy := -2; dy <= 2; dy++ {
					for dx := -2; dx <= 2; dx++ {
						nCellX := cellX + dx
						nCellY := cellY + dy

						// Skip if outside grid
						if nCellX < 0 || nCellX >= cellsPerCol || nCellY < 0 || nCellY >= cellsPerRow {
							continue
						}

						neighborDensity := g.density[nCellY][nCellX]

						// Only apply force if neighbor has lower density
						if neighborDensity < currentDensity {
							// Calculate density gradient
							densityGradient := currentDensity - neighborDensity

							// Calculate direction to cell center
							cellCenterX := (float64(nCellX) + 0.5) * cellSize
							cellCenterY := (float64(nCellY) + 0.5) * cellSize

							dx := cellCenterX - p.x[i]
							dy := cellCenterY - p.y[i]
							distSq := dx*dx + dy*dy

							// Skip if distance is too small
							if distSq < 1e-6 {
								continue
							}

							dist := math.Sqrt(distSq)
							nx := dx / dist
							ny := dy / dist

							// Calculate force
							forceMagnitude := densityGradient * 0.75

							// Accumulate forces
							netForceX += nx * forceMagnitude
							netForceY += ny * forceMagnitude
						} else if neighborDensity > currentDensity {
							// Calculate density gradient
							densityGradient := neighborDensity - currentDensity

							// Calculate direction to cell center
							cellCenterX := (float64(nCellX) + 0.5) * cellSize
							cellCenterY := (float64(nCellY) + 0.5) * cellSize

							dx := cellCenterX - p.x[i]
							dy := cellCenterY - p.y[i]
							distSq := dx*dx + dy*dy

							// Skip if distance is too small
							if distSq < 1e-6 {
								continue
							}

							dist := math.Sqrt(distSq)
							nx := dx / dist
							ny := dy / dist

							// Calculate force
							forceMagnitude := densityGradient * 0.75

							// Accumulate forces
							netForceX -= nx * forceMagnitude
							netForceY -= ny * forceMagnitude
						}
					}
				}

				// Apply net force
				p.vx[i] += netForceX
				p.vy[i] += netForceY
			}
		}(startIdx, endIdx)
	}
	wg.Wait()
}

// Draw renders particles with color based on velocity.
func (p *ParticleSOA) Draw(screen *ebiten.Image) {
	for i := 0; i < len(p.x); i++ {
		speed := math.Sqrt(p.vx[i]*p.vx[i] + p.vy[i]*p.vy[i])
		normalizedSpeed := math.Min(speed/150.0, 1.0)
		r := uint8(255 * normalizedSpeed)
		b := uint8(255 * (1.0 - normalizedSpeed))
		clr := color.RGBA{r, 0, b, 255}
		drawCircle(screen, int(p.x[i]), int(p.y[i]), ballRadius, clr)
	}
}

// Lower initial velocity for more stable simulation
func (p *ParticleSOA) InitParticles(numParticles int, screenWidth int, screenHeight int) {
	p.x = make([]float64, numParticles)
	p.y = make([]float64, numParticles)
	p.vx = make([]float64, numParticles)
	p.vy = make([]float64, numParticles)

	// Calculate grid dimensions
	gridSize := int(math.Sqrt(float64(numParticles)))
	if gridSize*gridSize < numParticles { // Adjust gridSize if numParticles is not a perfect square
		gridSize++
	}

	// Calculate spacing between particles, ensuring gridSize > 1 if numParticles > 1
	marginX := float64(screenWidth) * 0.1 // 10% margin
	marginY := float64(screenHeight) * 0.1
	effectiveScreenWidth := float64(screenWidth) - 2*marginX
	effectiveScreenHeight := float64(screenHeight) - 2*marginY

	spacingX := 0.0
	spacingY := 0.0
	if gridSize > 1 {
		spacingX = effectiveScreenWidth / float64(gridSize-1)
		spacingY = effectiveScreenHeight / float64(gridSize-1)
	} else if numParticles == 1 { // Center the single particle
		marginX = float64(screenWidth) / 2.0
		marginY = float64(screenHeight) / 2.0
	}

	// Place particles in a grid
	index := 0
	for i := 0; i < gridSize; i++ {
		for j := 0; j < gridSize; j++ {
			if index >= numParticles {
				break // Stop if we've placed all requested particles
			}
			// Handle single particle case or grid placement
			if gridSize > 1 {
				p.x[index] = marginX + float64(j)*spacingX + (rand.Float64()*2-1)*5 // Smaller jitter
				p.y[index] = marginY + float64(i)*spacingY + (rand.Float64()*2-1)*5
			} else {
				p.x[index] = marginX
				p.y[index] = marginY
			}

			p.vx[index] = (rand.Float64()*2 - 1) * 10 // Much smaller initial velocity
			p.vy[index] = (rand.Float64()*2 - 1) * 10
			index++
		}
		if index >= numParticles {
			break // Exit outer loop too
		}
	}
}

type Game struct {
	p             ParticleSOA
	grid          ParticleGrid
	gravity       bool
	spacePressed  bool      // Track if space was already pressed
	gravityToggle time.Time // Time when gravity was last toggled
}

func (p *ParticleSOA) calculateTotalVelocity() float64 {
	for i := 0; i < len(p.x); i++ {
		if count < 256 {
			totalVelocity[count] = math.Sqrt(p.vx[i]*p.vx[i] + p.vy[i]*p.vy[i])
			count++
		} else {
			count = 0
		}
	}
	// Calculate the average velocity
	total := 0.0
	for i := 0; i < len(totalVelocity); i++ {
		total += totalVelocity[i]
	}
	average := total / float64(len(totalVelocity))
	return average
}

func GraphTotalVelocity(velocities *[256]float64, screen *ebiten.Image) {
	graphWidth := 256
	graphHeight := 100
	offsetX := 10
	offsetY := screenHeight - graphHeight - 10

	// Draw the graph background
	for y := 0; y < graphHeight; y++ {
		for x := 0; x < graphWidth; x++ {
			screen.Set(offsetX+x, offsetY+y, color.RGBA{50, 50, 50, 255}) // Dark gray background
		}
	}

	// Find the maximum velocity for scaling
	maxVelocity := 0.0
	for _, v := range velocities {
		if v > maxVelocity {
			maxVelocity = v
		}
	}
	if maxVelocity == 0 {
		maxVelocity = 1 // Avoid division by zero
	}

	// Draw the velocity graph
	for i := 1; i < graphWidth; i++ {
		x1 := offsetX + i - 1
		y1 := offsetY + graphHeight - int((velocities[(count+i-1)%256]/maxVelocity)*float64(graphHeight))
		x2 := offsetX + i
		y2 := offsetY + graphHeight - int((velocities[(count+i)%256]/maxVelocity)*float64(graphHeight))

		// Draw a line between (x1, y1) and (x2, y2)
		drawLine(screen, x1, y1, x2, y2, color.RGBA{0, 255, 0, 255}) // Green line
	}
}

func (p *ParticleSOA) ApplyForce(velocity float64, posX, posY float64) {
	for i := 0; i < len(p.x); i++ {
		dx := p.x[i] - posX
		dy := p.y[i] - posY
		distSq := dx*dx + dy*dy

		p.vx[i] += (dx / distSq) * velocity
		p.vy[i] += (dy / distSq) * velocity
	}
}

func drawLine(screen *ebiten.Image, x1, y1, x2, y2 int, clr color.Color) {
	dx := int(math.Abs(float64(x2 - x1)))
	dy := int(math.Abs(float64(y2 - y1)))
	sx := -1
	if x1 < x2 {
		sx = 1
	}
	sy := -1
	if y1 < y2 {
		sy = 1
	}
	err := dx - dy

	for {
		screen.Set(x1, y1, clr)
		if x1 == x2 && y1 == y2 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x1 += sx
		}
		if e2 < dx {
			err += dx
			y1 += sy
		}
	}
}

func (g *Game) Update() error {
	g.p.Update(fixedDt, &g.grid, g.gravity)

	// Check for mouse input and apply force
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		g.p.ApplyForce(400, float64(x), float64(y))
	}
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		x, y := ebiten.CursorPosition()
		g.p.ApplyForce(-400, float64(x), float64(y))
	}

	// Improved space key handling with debouncing
	if ebiten.IsKeyPressed(ebiten.KeySpace) {
		// Only toggle if space wasn't already pressed and sufficient time has passed
		if !g.spacePressed && time.Since(g.gravityToggle) > 250*time.Millisecond {
			g.gravity = !g.gravity
			g.gravityToggle = time.Now()
		}
		g.spacePressed = true
	} else {
		g.spacePressed = false
	}

	return nil
}

func (g *ParticleGrid) DrawDensity(screen *ebiten.Image) {
	const cellWidth = screenWidth / cellsPerCol
	const cellHeight = screenHeight / cellsPerRow

	// Find the maximum density for scaling
	maxDensity := 0.0
	for i := 0; i < cellsPerRow; i++ {
		for j := 0; j < cellsPerCol; j++ {
			if g.density[i][j] > maxDensity {
				maxDensity = g.density[i][j]
			}
		}
	}

	// Avoid division by zero if all densities are zero
	if maxDensity == 0 {
		maxDensity = 1.0
	}

	// Draw the density heatmap
	for i := 0; i < cellsPerRow; i++ {
		for j := 0; j < cellsPerCol; j++ {
			density := g.density[i][j]
			normalizedDensity := density / maxDensity // Normalize density
			r := uint8(255 * normalizedDensity)
			b := uint8(255*(1.0-normalizedDensity)) >> 2
			clr := color.RGBA{r, 0, b, 128} // Gradient from blue to red with transparency

			// Fill the cell with the corresponding color
			xStart := j * cellWidth
			yStart := i * cellHeight
			for y := 0; y < cellHeight; y++ {
				for x := 0; x < cellWidth; x++ {
					screen.Set(xStart+x, yStart+y, clr)
				}
			}
		}
	}
}

func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.Black)
	// g.grid.DrawDensity(screen) // Draw the density heatmap
	g.p.Draw(screen) // Draw the particles
	// GraphTotalVelocity(&totalVelocity, screen)
	averageVelocity := g.p.calculateTotalVelocity()
	ebitenutil.DebugPrint(screen, fmt.Sprintf("Average Velocity: %.2f", averageVelocity))
	fps := ebiten.CurrentFPS()
	ebitenutil.DebugPrintAt(screen, fmt.Sprintf("FPS: %0.2f", fps), 10, 10)
	tps := ebiten.CurrentTPS()
	ebitenutil.DebugPrintAt(screen, fmt.Sprintf("TPS: %0.2f", tps), 10, 30)
	orderedKeys := []string{
		"ApplyDamping",
		"ResolveCollisions",
		"ResolveBoundaries",
		"ClearGrid",
		"SetParticleGrid",
		"CalculateDensity",
		"GaussianBlurDensity",
		"ApplyDensityForces",
	}

	// Display timings in the defined order
	for i, name := range orderedKeys {
		duration, exists := times[name]
		if exists {
			ebitenutil.DebugPrintAt(screen, fmt.Sprintf("%s: %0.2f ms", name, float64(duration.Microseconds())/1000.0), 10, 50+(i*20))
		}
	}
	ebitenutil.DebugPrintAt(screen, "Gravity: "+fmt.Sprintf("%t", g.gravity), 10, 50+(len(orderedKeys)*20))
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

// drawCircle draws a filled circle at (cx, cy) with given radius and color.
func drawCircle(dst *ebiten.Image, cx, cy, r int, clr color.Color) {
	rSq := r * r
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			if dx*dx+dy*dy <= rSq {
				px, py := cx+dx, cy+dy
				// Optional: Add bounds check for safety
				// if px >= 0 && px < screenWidth && py >= 0 && py < screenHeight {
				dst.Set(px, py, clr)
				// }
			}
		}
	}
}

var totalVelocity [256]float64
var count int

func main() {
	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowTitle("Velocity Colored Particles")

	// update TPS to 60 for smoother graphics
	ebiten.SetTPS(480)

	particles := &ParticleSOA{}
	// Use a perfect square for a nice grid, e.g., 100 for 10x10, or adjust as needed
	particles.InitParticles(4000, screenWidth, screenHeight) // Using fewer particles for stability

	// Initialize the particle grid
	particleGrid := &ParticleGrid{}
	particleGrid.ClearGrid()
	SetParticleGrid(particles, particleGrid)
	particleGrid.CalculateDensity()
	particleGrid.GaussianBlurDensity()

	game := &Game{
		p:             *particles,
		grid:          *particleGrid,
		spacePressed:  false,
		gravityToggle: time.Now().Add(-1 * time.Second), // Initialize with past time
	}

	if err := ebiten.RunGame(game); err != nil {
		log.Fatal(err)
	}
}
