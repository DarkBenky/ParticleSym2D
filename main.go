package main

import (
	"fmt"
	"image/color"
	"log"
	"math"
	"math/rand/v2"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

const (
	screenWidth   = 640
	screenHeight  = 480
	ballRadius    = 3
	maxSpeedColor = 50.0       // Adjust this value based on typical particle speeds
	fixedDt       = 1.0 / 30.0 // Fixed time step for stability
)

type ParticleSOA struct {
	x  []float64
	y  []float64
	vx []float64
	vy []float64
}

func (p *ParticleSOA) ApplyGravity(dt float64) {
	gravityEffect := 98.1 * dt * 20 // Gravity constant scaled by dt
	for i := 0; i < len(p.vy); i++ {
		p.vy[i] += gravityEffect
	}
}

func (p *ParticleSOA) ApplyDamping() {
	damping := 0.97 // Stronger damping for stability
	for i := 0; i < len(p.vx); i++ {
		p.vx[i] *= damping
		p.vy[i] *= damping
	}
}

func (p *ParticleSOA) ResolveCollisions() {
	numParticles := len(p.x)

	// Resolve positional overlaps
	for i := 0; i < numParticles; i++ {
		for j := i + 1; j < numParticles; j++ {
			dx := p.x[j] - p.x[i]
			dy := p.y[j] - p.y[i]
			distSq := dx*dx + dy*dy

			minDist := 2.0 * ballRadius
			if distSq < minDist*minDist && distSq > 1e-9 {
				dist := math.Sqrt(distSq)
				nx := dx / dist
				ny := dy / dist

				overlap := minDist - dist
				correction := 0.3 * overlap
				p.x[i] -= nx * correction * 0.5
				p.y[i] -= ny * correction * 0.5
				p.x[j] += nx * correction * 0.5
				p.y[j] += ny * correction * 0.5
			}
		}
	}

	// Resolve velocity changes
	for i := 0; i < numParticles; i++ {
		for k := i + 1; k < numParticles; k++ {
			dx := p.x[k] - p.x[i]
			dy := p.y[k] - p.y[i]
			distSq := dx*dx + dy*dy

			minDist := 2.0 * ballRadius
			if distSq < minDist*minDist && distSq > 1e-9 {
				dist := math.Sqrt(distSq)
				nx := dx / dist
				ny := dy / dist

				rvx := p.vx[i] - p.vx[k]
				rvy := p.vy[i] - p.vy[k]
				velAlongNormal := rvx*nx + rvy*ny

				if velAlongNormal < 0 {
					restitution := 0.35
					j := -(1.0 + restitution) * velAlongNormal
					impulseX := j * nx * 0.5
					impulseY := j * ny * 0.5

					p.vx[i] -= impulseX
					p.vy[i] -= impulseY
					p.vx[k] += impulseX
					p.vy[k] += impulseY
				}
			}
		}
	}
}

// ResolveBoundaries handles particle-boundary collisions with energy conservation.
func (p *ParticleSOA) ResolveBoundaries() {
	restitution := 0.7
	for i := 0; i < len(p.x); i++ {
		if p.x[i] < ballRadius {
			p.x[i] = ballRadius
			p.vx[i] *= -restitution
		} else if p.x[i] > screenWidth-ballRadius {
			p.x[i] = screenWidth - ballRadius
			p.vx[i] *= -restitution
		}
		if p.y[i] < ballRadius {
			p.y[i] = ballRadius
			p.vy[i] *= -restitution
		} else if p.y[i] > screenHeight-ballRadius {
			p.y[i] = screenHeight - ballRadius
			p.vy[i] *= -restitution
		}
	}
}

// Update integrates the physics simulation for one frame.
func (p *ParticleSOA) Update(dt float64) {
	p.ApplyGravity(dt)
	p.ApplyDamping()
	for i := 0; i < len(p.x); i++ {
		p.x[i] += p.vx[i] * dt
		p.y[i] += p.vy[i] * dt
	}
	p.ResolveCollisions()
	// Update velocities based on position changes
	for i := 0; i < len(p.x); i++ {
		p.vx[i] = (p.x[i] - p.x[i]) / dt
		p.vy[i] = (p.y[i] - p.y[i]) / dt
	}
	p.ResolveBoundaries()
	p.ApplyDamping()
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
				p.x[index] = marginX + float64(j)*spacingX + (rand.Float64()*2-1)*20
				p.y[index] = marginY + float64(i)*spacingY + (rand.Float64()*2-1)*20
			} else {
				p.x[index] = marginX
				p.y[index] = marginY
			}

			p.vx[index] = (rand.Float64()*2 - 1) * 1000 // Small initial random velocity
			p.vy[index] = (rand.Float64()*2 - 1) * 1000
			index++
		}
		if index >= numParticles {
			break // Exit outer loop too
		}
	}
}

type Game struct {
	p ParticleSOA
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
	g.p.Update(fixedDt)
	// check for mouse input and apply force
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		g.p.ApplyForce(5000, float64(x), float64(y))
	}
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) {
		x, y := ebiten.CursorPosition()
		g.p.ApplyForce(-5000, float64(x), float64(y))
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.Black)
	g.p.Draw(screen)
	GraphTotalVelocity(&totalVelocity, screen)
	averageVelocity := g.p.calculateTotalVelocity()
	ebitenutil.DebugPrint(screen, fmt.Sprintf("Average Velocity: %.2f", averageVelocity))
	fps := ebiten.CurrentFPS()
	ebitenutil.DebugPrintAt(screen, fmt.Sprintf("FPS: %0.2f", fps), 10, 10)
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
	ebiten.SetMaxTPS(90)

	particles := &ParticleSOA{}
	// Use a perfect square for a nice grid, e.g., 100 for 10x10, or adjust as needed
	particles.InitParticles(3000, screenWidth, screenHeight)

	game := &Game{
		p: *particles,
	}

	if err := ebiten.RunGame(game); err != nil {
		log.Fatal(err)
	}
}
