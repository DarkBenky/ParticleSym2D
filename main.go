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
	screenWidth      = 640
	screenHeight     = 480
	ballRadius       = 3
	physicIterations = 4
	maxSpeedColor    = 50.0 // Adjust this value based on typical particle speeds
)

type ParticleSOA struct {
	x  []float64
	y  []float64
	vx []float64
	vy []float64
}

func (p *ParticleSOA) ApplyGravity() {
	for i := 0; i < len(p.x); i++ {
		p.vy[i] += 0.981 // Gravity acceleration
	}
}

func (p *ParticleSOA) UpdatePosition(height int, width int) {
	restitution := 0.75 // Bounciness factor for walls
	radiusF := float64(ballRadius)
	widthF := float64(width)
	heightF := float64(height)

	for i := 0; i < len(p.x); i++ {
		// Update position
		p.x[i] += p.vx[i]
		p.y[i] += p.vy[i]

		// Boundary collision detection and response
		if p.x[i] < radiusF {
			p.x[i] = radiusF
			p.vx[i] *= -restitution
		} else if p.x[i] > widthF-radiusF {
			p.x[i] = widthF - radiusF
			p.vx[i] *= -restitution
		}

		if p.y[i] < radiusF {
			p.y[i] = radiusF
			p.vy[i] *= -restitution
		} else if p.y[i] > heightF-radiusF {
			p.y[i] = heightF - radiusF
			p.vy[i] *= -restitution
		}
	}
}

func CollideParticles(px1, py1, vx1, vy1, px2, py2, vx2, vy2 float64) (newVx1, newVy1, newVx2, newVy2, dxCorrection, dyCorrection float64) {
	newVx1, newVy1, newVx2, newVy2 = vx1, vy1, vx2, vy2
	dx := px2 - px1
	dy := py2 - py1
	distSq := dx*dx + dy*dy

	minDist := 2.0 * ballRadius
	if distSq < minDist*minDist && distSq > 1e-9 {
		dist := math.Sqrt(distSq)
		nx := dx / dist
		ny := dy / dist

		rvx := vx1 - vx2
		rvy := vy1 - vy2
		velAlongNormal := rvx*nx + rvy*ny
		if velAlongNormal > 0 {
			return
		}

		restitution := 0.75
		j := -(1.0 + restitution) * velAlongNormal
		impulseX := j * nx * 0.5
		impulseY := j * ny * 0.5

		newVx1 -= impulseX
		newVy1 -= impulseY
		newVx2 += impulseX
		newVy2 += impulseY

		// 🔧 Positional correction (prevent sticking and energy gain)
		overlap := minDist - dist
		correction := 0.5 * overlap
		dxCorrection = nx * correction
		dyCorrection = ny * correction
	}

	return newVx1, newVy1, newVx2, newVy2, dxCorrection * 0.75, dyCorrection * 0.75
}

func (p *ParticleSOA) CollideParticles() {
	numParticles := len(p.x)
	for i := 0; i < numParticles; i++ {
		for j := i + 1; j < numParticles; j++ {
			// Pass current positions and velocities to the collision function
			newVx1, newVy1, newVx2, newVy2, dxCorrection, dyCorrection := CollideParticles(
				p.x[i], p.y[i], p.vx[i], p.vy[i],
				p.x[j], p.y[j], p.vx[j], p.vy[j],
			)
			// Update velocities
			p.vx[i], p.vy[i] = newVx1, newVy1
			p.vx[j], p.vy[j] = newVx2, newVy2
			// Apply positional correction
			p.x[i] += dxCorrection
			p.y[i] += dyCorrection
			p.x[j] -= dxCorrection
			p.y[j] -= dyCorrection
		}
	}
}

// Draw renders particles with color based on velocity.
func (p *ParticleSOA) Draw(screen *ebiten.Image) {
	for i := 0; i < len(p.x); i++ {
		// Calculate velocity magnitude (speed)
		speed := math.Sqrt(p.vx[i]*p.vx[i] + p.vy[i]*p.vy[i])

		// Normalize speed (0 to 1 range) based on maxSpeedColor
		normalizedSpeed := math.Min(speed/maxSpeedColor, 1.0) // Clamp at 1.0

		// Interpolate color between blue (slow) and red (fast)
		r := uint8(255 * normalizedSpeed)
		g := uint8(0) // Keep green at 0 for blue-to-red transition
		b := uint8(255 * (1.0 - normalizedSpeed))

		clr := color.RGBA{r, g, b, 255}

		drawCircle(screen, int(p.x[i]), int(p.y[i]), ballRadius, clr)
	}
}

// func (p *ParticleSOA) Update() {
// 	// Note: This update loop doesn't use dt, which can lead to frame-rate dependent physics.
// 	// Consider adding dt as shown in previous examples for better stability.
// 	p.ApplyGravity()
// 	p.CollideParticles()
// 	p.UpdatePosition(screenHeight, screenWidth)
// }

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
				p.x[index] = marginX + float64(j)*spacingX
				p.y[index] = marginY + float64(i)*spacingY
			} else {
				p.x[index] = marginX
				p.y[index] = marginY
			}

			p.vx[index] = rand.Float64()*2 - 1 // Small initial random velocity
			p.vy[index] = rand.Float64()*2 - 1
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

func (p *ParticleSOA) ApplyDamping() {
	damping := 0.99 // Damping factor (1.0 = no damping, <1.0 = gradual slowdown)
	for i := 0; i < len(p.vx); i++ {
		p.vx[i] *= damping
		p.vy[i] *= damping
	}
}

func (g *Game) Update() error {
	// g.p.ApplyGravity()
	g.p.CollideParticles()
	g.p.UpdatePosition(screenHeight, screenWidth)
	g.p.ApplyDamping()
	return nil
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

func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.Black)
	g.p.Draw(screen)
	GraphTotalVelocity(&totalVelocity, screen)
	averageVelocity := g.p.calculateTotalVelocity()
	ebitenutil.DebugPrint(screen, fmt.Sprintf("Average Velocity: %.2f", averageVelocity))
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

	particles := &ParticleSOA{}
	// Use a perfect square for a nice grid, e.g., 100 for 10x10, or adjust as needed
	particles.InitParticles(100, screenWidth, screenHeight)

	game := &Game{
		p: *particles,
	}

	if err := ebiten.RunGame(game); err != nil {
		log.Fatal(err)
	}
}
