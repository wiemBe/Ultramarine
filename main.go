package main

import (
	"container/heap"
	"database/sql"
	"errors"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	_ "github.com/mattn/go-sqlite3" // SQLite driver
)

// --- Configuration ---
const (
	dbPath         = "./robot_tasks.db" // Database file name
	maxActiveTasks = 5                  // Max tasks in pending or assigned state before rejecting new ones
	serverPort     = ":8080"            // Port for the web server
)

// --- Task Status Constants ---
const (
	StatusPending   = "pending"
	StatusAssigned  = "assigned"
	StatusCompleted = "completed"
	StatusFailed    = "failed"
)

// --- Map Constants ---
const (
	CellFree     = 0 // Represents walkable space on the map
	CellObstacle = 1 // Represents an obstacle on the map
)

// --- Robot Orientation Constants ---
const (
	OrientationNorth = 0
	OrientationEast  = 1
	OrientationSouth = 2
	OrientationWest  = 3
)

// --- Robot Command Constants (Sent to Robot) ---
const (
	CmdForward     = "FORWARD(1)"    // Move forward one grid unit
	CmdTurnLeft    = "TURN_LEFT(90)" // Turn 90 degrees left
	CmdTurnRight   = "TURN_RIGHT(90)" // Turn 90 degrees right
	CmdNone        = "NO_COMMAND"     // No command currently available (e.g., waiting for init)
	CmdComplete    = "PATH_COMPLETE"  // Indicates successful path completion
	CmdRecalculate = "RECALCULATE"    // Deprecated/Optional: Ask robot to report status again for replan
	CmdError       = "ERROR"          // Indicates a server-side error occurred (e.g., pathfinding failed)
)

// --- Robot Status Constants (Received from Robot) ---
const (
	StatusInit     = "INIT"     // Robot reporting initial state for the task
	StatusDone     = "DONE"     // Robot finished the last command successfully
	StatusObstacle = "OBSTACLE" // Robot encountered an unexpected obstacle
)

// --- Map Data (Example Grid) ---
// 0 = free, 1 = obstacle
// Remember: Y typically increases downwards in 2D array indexing
var worldMap = [][]int{
	{0, 0, 0, 1, 0, 0},
	{0, 1, 0, 0, 0, 0},
	{0, 0, 1, 0, 1, 0},
	{0, 0, 0, 0, 1, 0},
	{1, 1, 0, 0, 0, 0},
	{0, 0, 0, 1, 0, 0},
}
var mapWidth = len(worldMap[0])
var mapHeight = len(worldMap)

// Example mapping from Station ID (used in tasks) to Map Coordinates (Point)
var stationCoords = map[int]Point{
	1: {X: 0, Y: 0}, // Station 1 at top-left
	2: {X: 5, Y: 0}, // Station 2 at top-right
	3: {X: 0, Y: 5}, // Station 3 at bottom-left
	4: {X: 5, Y: 5}, // Station 4 at bottom-right
	// Add more stations corresponding to your map
}

// --- Data Structures ---

// Task represents a job stored in the database
type Task struct {
	ID              int64     `json:"id"`
	Station         int       `json:"station"` // Target station ID
	Status          string    `json:"status"`  // pending, assigned, completed, failed
	AssignedRobotID *string   `json:"assigned_robot_id,omitempty"`
	CreatedAt       time.Time `json:"created_at"`
	UpdatedAt       time.Time `json:"updated_at"`
}

// Point represents a coordinate on the grid map
type Point struct {
	X int
	Y int
}

// RobotTaskState tracks the live state of an assigned task being executed by a robot
type RobotTaskState struct {
	TaskID              int64
	TargetStation       int
	TargetPoint         Point    // Target X,Y coordinates
	CurrentPosition     Point    // Last reported X,Y position from the robot
	CurrentOrientation  int      // Last reported orientation (0=N, 1=E, 2=S, 3=W)
	PathCommands        []string // Sequence of commands generated from A* path
	CurrentCommandIndex int      // Index of the next command to be sent
	NeedsPathRecalc     bool     // Flag set if path needs recalculation (e.g., obstacle)
	IsInitialized       bool     // Flag set after receiving the first INIT status report
	Mutex               sync.Mutex // Protects concurrent access to this specific state instance
}

// --- Global State ---
var db *sql.DB // Database connection handle

// Map: Task ID -> *RobotTaskState (stores active path/command state for assigned tasks)
var robotStates = make(map[int64]*RobotTaskState)
var robotStatesMutex sync.RWMutex // Mutex to protect the robotStates map itself

// --- Main Application ---
func main() {
	initDB() // Initialize DB connection and schema
	defer db.Close()

	// Initialize robot state tracking map
	robotStates = make(map[int64]*RobotTaskState)

	// Setup Gin web server
	server := gin.Default()
	server.LoadHTMLGlob("templates/*") // Load HTML templates if needed

	// API Route Definitions
	tasksGroup := server.Group("/tasks")
	{
		// Task creation and listing
		tasksGroup.POST("", handleAddTask)          // Add a new task to the queue
		tasksGroup.GET("", handleGetTasks)           // Get list of all tasks (can filter later)
		tasksGroup.POST("/next", handleGetNextTask)  // Robot requests the next available task assignment

		// Routes specific to an ongoing task (identified by :id)
		taskIDGroup := tasksGroup.Group("/:id")
		{
			// Final task outcomes reported by robot or triggered internally
			taskIDGroup.POST("/complete", handleCompleteTask) // Mark task as completed
			taskIDGroup.POST("/fail", handleFailTask)         // Mark task as failed

			// Robot interaction during task execution
			taskIDGroup.POST("/report_status", handleReportStatus) // Robot reports its current state (pos, orientation, last cmd status)
			taskIDGroup.GET("/next_command", handleGetNextCommand)  // Robot requests the next navigation command
		}
	}

	// Route for serving the frontend (optional)
	server.GET("/", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "index.html", gin.H{
			"Title": "Robot Task Queue",
		})
	})

	log.Printf("Server starting on port %s...", serverPort)
	if err := server.Run(serverPort); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}

// --- Route Handlers ---

// handleAddTask: Adds a new task to the queue with 'pending' status.
func handleAddTask(ctx *gin.Context) {
	var input struct {
		Station int `json:"station" binding:"required"`
	}

	if err := ctx.ShouldBindJSON(&input); err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid input", "details": err.Error()})
		return
	}

	// Check if the target station exists in our coordinate map
	if _, ok := stationCoords[input.Station]; !ok {
		log.Printf("Attempted to add task for unknown station ID: %d", input.Station)
		ctx.JSON(http.StatusBadRequest, gin.H{"error": fmt.Sprintf("Invalid station ID: %d. Not defined in server configuration.", input.Station)})
		return
	}


	// Check active task limit
	var activeCount int
	err := db.QueryRow("SELECT COUNT(*) FROM tasks WHERE status = ? OR status = ?", StatusPending, StatusAssigned).Scan(&activeCount)
	if err != nil {
		log.Printf("Error counting active tasks: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error checking queue limit"})
		return
	}

	if activeCount >= maxActiveTasks {
		ctx.JSON(http.StatusServiceUnavailable, gin.H{"error": "Queue is full", "limit": maxActiveTasks}) // 503 Service Unavailable or 429 Too Many Requests
		return
	}

	// Insert new task into DB
	stmt, err := db.Prepare("INSERT INTO tasks (station, status) VALUES (?, ?)")
	if err != nil {
		log.Printf("Error preparing insert statement: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error preparing insert"})
		return
	}
	defer stmt.Close()

	res, err := stmt.Exec(input.Station, StatusPending)
	if err != nil {
		log.Printf("Error inserting new task: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error inserting task"})
		return
	}

	newID, _ := res.LastInsertId()

	// Fetch the newly created task to return it
	var createdTask Task
	err = db.QueryRow(`SELECT id, station, status, assigned_robot_id, created_at, updated_at FROM tasks WHERE id = ?`, newID).Scan(
		&createdTask.ID, &createdTask.Station, &createdTask.Status, &createdTask.AssignedRobotID, &createdTask.CreatedAt, &createdTask.UpdatedAt,
	)
	if err != nil {
		log.Printf("Error fetching created task (ID: %d): %v", newID, err)
		// Still return success, just without the echo of the created object
		ctx.JSON(http.StatusCreated, gin.H{"message": "Task created successfully", "id": newID})
		return
	}

	ctx.JSON(http.StatusCreated, createdTask)
}

// handleGetTasks: Retrieves a list of tasks (all statuses by default).
func handleGetTasks(ctx *gin.Context) {
	// TODO: Add query parameters for filtering (e.g., /tasks?status=pending)
	rows, err := db.Query(`SELECT id, station, status, assigned_robot_id, created_at, updated_at FROM tasks ORDER BY id`)
	if err != nil {
		log.Printf("Error querying tasks: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error fetching tasks"})
		return
	}
	defer rows.Close()

	var tasks []Task
	for rows.Next() {
		var task Task
		var assignedRobotID sql.NullString // Handle nullable column
		if err := rows.Scan(&task.ID, &task.Station, &task.Status, &assignedRobotID, &task.CreatedAt, &task.UpdatedAt); err != nil {
			log.Printf("Error scanning task row: %v", err)
			ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error processing tasks"})
			return // Stop processing if one row fails
		}
		if assignedRobotID.Valid {
			task.AssignedRobotID = &assignedRobotID.String
		}
		tasks = append(tasks, task)
	}

	// Check for errors during row iteration
	if err := rows.Err(); err != nil {
		log.Printf("Error after iterating task rows: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error reading tasks"})
		return
	}

	ctx.JSON(http.StatusOK, gin.H{"tasks": tasks})
}

// handleGetNextTask: Robot requests a task; server assigns the oldest pending one.
func handleGetNextTask(ctx *gin.Context) {
	// Begin DB transaction for atomic find-and-update
	tx, err := db.Begin()
	if err != nil {
		log.Printf("Error beginning transaction for getNextTask: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error (begin tx)"})
		return
	}
	defer tx.Rollback() // Ensure rollback on any error before commit

	// Find the oldest pending task
	var nextTask Task
	var assignedRobotID sql.NullString // Placeholder
	err = tx.QueryRow(`SELECT id, station, status, assigned_robot_id, created_at, updated_at
	                   FROM tasks WHERE status = ? ORDER BY id LIMIT 1`, StatusPending).Scan(
		&nextTask.ID, &nextTask.Station, &nextTask.Status, &assignedRobotID, &nextTask.CreatedAt, &nextTask.UpdatedAt,
	)

	if err == sql.ErrNoRows {
		// This is expected when the queue is empty, not an error
		ctx.JSON(http.StatusNotFound, gin.H{"message": "No pending tasks available"})
		return
	} else if err != nil {
		log.Printf("Error querying for next pending task: %v", err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error finding next task"})
		return
	}

	// Task found, update its status to assigned in the DB
	_, err = tx.Exec("UPDATE tasks SET status = ? WHERE id = ?", StatusAssigned, nextTask.ID)
	if err != nil {
		log.Printf("Error updating task status to assigned (Task %d): %v", nextTask.ID, err)
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error assigning task"})
		return
	}

	// Get target coordinates from our station map
	targetPoint, ok := stationCoords[nextTask.Station]
	if !ok {
		// This should have been caught during AddTask, but double-check
		log.Printf("CRITICAL Error: Assigned task %d for unknown station %d", nextTask.ID, nextTask.Station)
		// Rollback the assignment as we cannot proceed
		tx.Rollback() // Explicit rollback before returning error
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": fmt.Sprintf("Configuration error: Station %d coordinates not found for assigned task", nextTask.Station)})
		return
	}

	// Create the live state tracker for this task
	newState := &RobotTaskState{
		TaskID:              nextTask.ID,
		TargetStation:       nextTask.Station,
		TargetPoint:         targetPoint,
		CurrentCommandIndex: 0,
		PathCommands:        nil,  // Path calculated only after robot reports its initial state
		NeedsPathRecalc:     true, // Requires path calculation
		IsInitialized:       false, // Waiting for the first "INIT" status report
		// CurrentPosition and CurrentOrientation are unknown until the first report
	}

	// Add state to the global map (protected by mutex)
	robotStatesMutex.Lock()
	robotStates[nextTask.ID] = newState
	robotStatesMutex.Unlock()

	// Commit the database transaction (task status is now 'assigned')
	if err := tx.Commit(); err != nil {
		log.Printf("Error committing transaction for task assignment (Task %d): %v", nextTask.ID, err)
		// Clean up the state we added if commit fails
		robotStatesMutex.Lock()
		delete(robotStates, nextTask.ID)
		robotStatesMutex.Unlock()
		ctx.JSON(http.StatusInternalServerError, gin.H{"error": "Database error committing assignment"})
		return
	}

	log.Printf("Assigned Task ID %d (Station %d) successfully\n", nextTask.ID, nextTask.Station)
	nextTask.Status = StatusAssigned // Update status in the object being returned
	ctx.JSON(http.StatusOK, nextTask)
}

// handleReportStatus: Robot reports its position, orientation, and last command status.
func handleReportStatus(ctx *gin.Context) {
	// Extract Task ID from URL path
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64)
	if err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID format in URL"})
		return
	}

	// Define expected JSON input structure
	var input struct {
		X             int    `json:"x" binding:"required"`
		Y             int    `json:"y" binding:"required"`
		Theta         int    `json:"theta"` // Orientation (make required if always sent)
		LastCmdStatus string `json:"last_cmd_status" binding:"required"` // INIT, DONE, OBSTACLE
	}

	// Bind and validate JSON input
	if err := ctx.ShouldBindJSON(&input); err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid input JSON", "details": err.Error()})
		return
	}

	// Validate Orientation if provided or required
	if input.Theta < OrientationNorth || input.Theta > OrientationWest {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid orientation (theta) value", "details": "Must be 0(N), 1(E), 2(S), or 3(W)"})
		return
	}
	// Validate LastCmdStatus
	inputStatusUpper := strings.ToUpper(input.LastCmdStatus)
	isValidStatus := inputStatusUpper == StatusInit || inputStatusUpper == StatusDone || inputStatusUpper == StatusObstacle
	if !isValidStatus {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid last_cmd_status value", "details": "Must be INIT, DONE, or OBSTACLE"})
		return
	}

	// Find the active state for this task ID (read lock)
	robotStatesMutex.RLock()
	state, exists := robotStates[taskID]
	robotStatesMutex.RUnlock()

	if !exists {
		log.Printf("Warning: Status report received for inactive/unknown Task ID: %d", taskID)
		ctx.JSON(http.StatusNotFound, gin.H{"error": "Task state not found (might be completed/failed or invalid ID)"})
		return
	}

	// Lock the specific task's state for modification
	state.Mutex.Lock()
	defer state.Mutex.Unlock()

	log.Printf("Status Report Task %d: Pos=(%d,%d), Theta=%d, LastCmd=%s",
		taskID, input.X, input.Y, input.Theta, inputStatusUpper)

	// Update the robot's known state
	state.CurrentPosition = Point{X: input.X, Y: input.Y}
	state.CurrentOrientation = input.Theta

	// Process based on the status reported
	switch inputStatusUpper {
	case StatusInit:
		state.IsInitialized = true   // Mark as initialized
		state.NeedsPathRecalc = true // Trigger path calculation on next command request
		state.CurrentCommandIndex = 0 // Ensure command index is reset

	case StatusDone:
		// Robot completed the command at the *previous* index successfully
		if state.PathCommands != nil && state.CurrentCommandIndex < len(state.PathCommands) {
			// Only increment if there are more commands expected
			state.CurrentCommandIndex++
		} else {
			// Reported DONE but server thinks path is already finished or wasn't calculated?
			log.Printf("Warning: Task %d reported DONE, but command index (%d) is at/beyond path length (%d). Path might be complete.",
				taskID, state.CurrentCommandIndex, len(state.PathCommands))
			// No index increment needed. Next /next_command will return PATH_COMPLETE.
		}
		state.NeedsPathRecalc = false // Continue with the current path

	case StatusObstacle:
		// Unexpected obstacle encountered
		log.Printf("Obstacle reported for Task %d at (%d,%d). Path recalculation needed.", taskID, input.X, input.Y)
		state.NeedsPathRecalc = true // Mark for replanning
		state.PathCommands = nil     // Invalidate the old command list
		state.CurrentCommandIndex = 0 // Reset command index for the new path
		// Optional TODO: Mark the reported cell (input.X, input.Y) as a temporary obstacle
		// in a separate data structure or even modify worldMap temporarily (carefully!).

	}

	ctx.JSON(http.StatusOK, gin.H{"message": "Status received"})
}

// handleGetNextCommand: Robot requests the next command for its assigned task.
func handleGetNextCommand(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64)
	if err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID format"})
		return
	}

	// Find the active state (read lock)
	robotStatesMutex.RLock()
	state, exists := robotStates[taskID]
	robotStatesMutex.RUnlock()

	if !exists {
		ctx.JSON(http.StatusNotFound, gin.H{"command": CmdError, "error": "Task state not found"})
		return
	}

	// Lock the specific task's state
	state.Mutex.Lock()
	defer state.Mutex.Unlock()

	// Check if we have received the initial position report yet
	if !state.IsInitialized {
		log.Printf("Task %d: Waiting for initial status report before path calculation.", taskID)
		ctx.JSON(http.StatusOK, gin.H{"command": CmdNone, "message": "Waiting for initial robot position report via /report_status"})
		return
	}


	// Calculate or recalculate path if needed
	if state.PathCommands == nil || state.NeedsPathRecalc {
		log.Printf("Calculating path for Task %d: From (%d,%d)@Theta:%d -> Station %d (%d,%d)",
			taskID, state.CurrentPosition.X, state.CurrentPosition.Y, state.CurrentOrientation,
			state.TargetStation, state.TargetPoint.X, state.TargetPoint.Y)

		// Perform A* pathfinding
		pathPoints, pathErr := findPathAStar(state.CurrentPosition, state.TargetPoint)
		if pathErr != nil {
			log.Printf("Error finding path for Task %d: %v", taskID, pathErr)
			// Should we fail the task now? Let's return error command first.
			ctx.JSON(http.StatusOK, gin.H{"command": CmdError, "error": fmt.Sprintf("Pathfinding failed: %v", pathErr)})
			// Optional: Consider calling internalFailTask(taskID) here.
			return
		}

		// Handle empty path result
		if len(pathPoints) == 0 {
			// This likely means start == goal
			if state.CurrentPosition == state.TargetPoint {
				log.Printf("Task %d: Start position equals target. Path complete.", taskID)
				go internalCompleteTask(taskID) // Complete task asynchronously
				ctx.JSON(http.StatusOK, gin.H{"command": CmdComplete})
				return
			} else {
				// A* returned empty path but not at goal - indicates unreachable
				log.Printf("Error: A* returned empty path for Task %d, but not at target.", taskID)
				ctx.JSON(http.StatusOK, gin.H{"command": CmdError, "error": "Path not found or target unreachable"})
				return
			}
		}

		// Convert the path points to robot commands
		commands, cmdErr := pathToCommands(pathPoints, state.CurrentOrientation)
		if cmdErr != nil {
			log.Printf("Error converting path to commands for Task %d: %v", taskID, cmdErr)
			ctx.JSON(http.StatusOK, gin.H{"command": CmdError, "error": fmt.Sprintf("Command generation failed: %v", cmdErr)})
			return
		}

		log.Printf("Task %d: Generated %d commands.", taskID, len(commands))
		state.PathCommands = commands
		state.CurrentCommandIndex = 0 // Start from the beginning
		state.NeedsPathRecalc = false // Path is now fresh

	} // End path calculation block

	// --- Return the appropriate command ---
	if state.PathCommands == nil {
		// Should not happen if logic above is correct, but safeguard
		log.Printf("Error: Task %d state has nil PathCommands after calculation attempt.", taskID)
		ctx.JSON(http.StatusInternalServerError, gin.H{"command": CmdError, "error": "Internal server error: Path unavailable"})
		return
	}

	if state.CurrentCommandIndex < len(state.PathCommands) {
		// Return the next command from the list
		nextCmd := state.PathCommands[state.CurrentCommandIndex]
		log.Printf("Task %d: Sending Command %d/%d: %s", taskID, state.CurrentCommandIndex+1, len(state.PathCommands), nextCmd)
		ctx.JSON(http.StatusOK, gin.H{"command": nextCmd})
		// NOTE: Index is incremented only when the robot reports "DONE" via /report_status
	} else {
		// All commands have been sent and presumably executed (robot reported DONE for the last one)
		log.Printf("Task %d: All path commands processed. Sending PATH_COMPLETE.", taskID)
		// Trigger final completion asynchronously
		go internalCompleteTask(taskID)
		ctx.JSON(http.StatusOK, gin.H{"command": CmdComplete})
	}
}

// handleCompleteTask: Finalizes task completion reported by robot or triggered internally.
func handleCompleteTask(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64)
	if err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID format"})
		return
	}

	if err := internalCompleteTask(taskID); err != nil {
		// Check if it was already completed/cleaned up, which isn't a server error
		if errors.Is(err, sql.ErrNoRows) || strings.Contains(err.Error(), "Task state not found") {
			ctx.JSON(http.StatusOK, gin.H{"message": "Task already marked complete or state cleared", "id": taskID})
		} else {
			ctx.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		}
		return
	}
	ctx.JSON(http.StatusOK, gin.H{"message": "Task marked as completed", "id": taskID})
}

// handleFailTask: Finalizes task failure reported by robot or triggered internally.
func handleFailTask(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64)
	if err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID format"})
		return
	}

	if err := internalFailTask(taskID); err != nil {
		if errors.Is(err, sql.ErrNoRows) || strings.Contains(err.Error(), "Task state not found") {
			ctx.JSON(http.StatusOK, gin.H{"message": "Task already marked failed or state cleared", "id": taskID})
		} else {
			ctx.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		}
		return
	}
	ctx.JSON(http.StatusOK, gin.H{"message": "Task marked as failed", "id": taskID})
}

// --- Internal Helper Functions (Update DB & Cleanup State) ---

// internalCompleteTask: Updates DB to completed and removes state from memory map.
func internalCompleteTask(taskID int64) error {
	log.Printf("Attempting internal completion for Task %d...", taskID)
	// Update DB status, only if not already completed
	res, err := db.Exec("UPDATE tasks SET status = ? WHERE id = ? AND status != ?", StatusCompleted, taskID, StatusCompleted)
	if err != nil {
		log.Printf("Error updating task %d status to completed in DB: %v", taskID, err)
		return fmt.Errorf("database error completing task: %w", err)
	}
	rowsAffected, _ := res.RowsAffected()
	if rowsAffected == 0 {
		log.Printf("DB: Task %d already completed or not found.", taskID)
		// Proceed to cleanup anyway, state might still exist if called multiple times
	} else {
		log.Printf("DB: Task %d marked as completed.", taskID)
	}

	// Clean up the in-memory state tracking
	robotStatesMutex.Lock()
	_, exists := robotStates[taskID]
	if exists {
		delete(robotStates, taskID)
		log.Printf("State: Cleaned up state for completed task %d.", taskID)
	} else {
		log.Printf("State: No active state found for completed task %d (already cleaned?).", taskID)
	}
	robotStatesMutex.Unlock()
	return nil // No error even if already cleaned up
}

// internalFailTask: Updates DB to failed and removes state from memory map.
func internalFailTask(taskID int64) error {
	log.Printf("Attempting internal failure marking for Task %d...", taskID)
	// Update DB status, only if not already failed
	res, err := db.Exec("UPDATE tasks SET status = ? WHERE id = ? AND status != ?", StatusFailed, taskID, StatusFailed)
	if err != nil {
		log.Printf("Error updating task %d status to failed in DB: %v", taskID, err)
		return fmt.Errorf("database error failing task: %w", err)
	}
	rowsAffected, _ := res.RowsAffected()
	if rowsAffected == 0 {
		log.Printf("DB: Task %d already failed or not found.", taskID)
	} else {
		log.Printf("DB: Task %d marked as failed.", taskID)
	}

	// Clean up the in-memory state tracking
	robotStatesMutex.Lock()
	_, exists := robotStates[taskID]
	if exists {
		delete(robotStates, taskID)
		log.Printf("State: Cleaned up state for failed task %d.", taskID)
	} else {
		log.Printf("State: No active state found for failed task %d (already cleaned?).", taskID)
	}
	robotStatesMutex.Unlock()
	return nil // No error even if already cleaned up
}

// --- A* Pathfinding Implementation ---

// aStarNode holds state for a node during A* search
type aStarNode struct {
	p        Point      // The coordinates of the node
	g, h, f  float64    // Cost values: g=cost from start, h=heuristic to goal, f=g+h
	parent   *aStarNode // The node we came from to reach this one
	index    int        // Index of the item in the priority queue (managed by heap interface)
}

// priorityQueue implements heap.Interface for aStarNode based on F cost
type priorityQueue []*aStarNode

func (pq priorityQueue) Len() int { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool { return pq[i].f < pq[j].f } // Min-heap
func (pq priorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *priorityQueue) Push(x interface{}) {
	n := len(*pq)
	node := x.(*aStarNode)
	node.index = n
	*pq = append(*pq, node)
}
func (pq *priorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	old[n-1] = nil  // avoid memory leak
	node.index = -1 // for safety
	*pq = old[0 : n-1]
	return node
}

// heuristic estimates the cost from point a to point b (Manhattan distance)
func heuristic(a, b Point) float64 {
	return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
}

// isValid checks if a point is within map bounds and not an obstacle
func isValid(p Point) bool {
	return p.X >= 0 && p.X < mapWidth && p.Y >= 0 && p.Y < mapHeight && worldMap[p.Y][p.X] == CellFree
}

// getNeighbors returns valid, walkable neighbor points for A*
func getNeighbors(p Point) []Point {
	neighbors := []Point{}
	// Define possible moves (North, East, South, West)
	moves := []Point{{X: 0, Y: -1}, {X: 1, Y: 0}, {X: 0, Y: 1}, {X: -1, Y: 0}}
	for _, move := range moves {
		neighbor := Point{X: p.X + move.X, Y: p.Y + move.Y}
		if isValid(neighbor) {
			neighbors = append(neighbors, neighbor)
		}
	}
	return neighbors
}

// findPathAStar implements the A* search algorithm
func findPathAStar(start, goal Point) ([]Point, error) {
	if !isValid(start) {
		return nil, fmt.Errorf("start point (%d,%d) is invalid", start.X, start.Y)
	}
	if !isValid(goal) {
		return nil, fmt.Errorf("goal point (%d,%d) is invalid", goal.X, goal.Y)
	}
	if start == goal {
		return []Point{}, nil // No path needed if already at goal
	}

	openSet := make(priorityQueue, 0)
	heap.Init(&openSet)

	closedSet := make(map[Point]bool)      // Points already evaluated
	nodeMap := make(map[Point]*aStarNode) // Stores nodes for quick lookup

	startNode := &aStarNode{p: start, g: 0, h: heuristic(start, goal)}
	startNode.f = startNode.g + startNode.h
	heap.Push(&openSet, startNode)
	nodeMap[start] = startNode

	for openSet.Len() > 0 {
		// Get node with the lowest F score
		current := heap.Pop(&openSet).(*aStarNode)

		// Goal reached?
		if current.p == goal {
			// Reconstruct path by backtracking through parents
			path := []Point{}
			temp := current
			for temp != nil {
				path = append(path, temp.p)
				temp = temp.parent
			}
			// Reverse the path to get start -> goal order
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path, nil // Success
		}

		// Mark current node as evaluated
		closedSet[current.p] = true

		// Explore neighbors
		for _, neighborPoint := range getNeighbors(current.p) {
			if closedSet[neighborPoint] {
				continue // Skip already evaluated neighbors
			}

			// Calculate tentative G score for neighbor
			gScore := current.g + 1.0 // Assuming cost of 1 for each grid step

			neighborNode, exists := nodeMap[neighborPoint]

			// If neighbor not in open set or this path is better
			if !exists || gScore < neighborNode.g {
				if !exists {
					neighborNode = &aStarNode{p: neighborPoint}
					nodeMap[neighborPoint] = neighborNode
				}
				// Update neighbor's details
				neighborNode.parent = current
				neighborNode.g = gScore
				neighborNode.h = heuristic(neighborPoint, goal)
				neighborNode.f = neighborNode.g + neighborNode.h

				if !exists {
					heap.Push(&openSet, neighborNode) // Add to open set
				} else {
					heap.Fix(&openSet, neighborNode.index) // Update position in priority queue
				}
			}
		}
	}

	// Open set is empty but goal was not reached
	return nil, errors.New("path not found (open set empty)")
}

// --- Command Generation ---

// pathToCommands converts a list of path points into robot commands (FORWARD, TURN_LEFT, TURN_RIGHT)
func pathToCommands(path []Point, startOrientation int) ([]string, error) {
	if len(path) < 2 {
		// Path is just the starting point or empty, no movement needed
		return []string{}, nil
	}

	commands := []string{}
	currentOrientation := startOrientation
	currentPos := path[0]

	for i := 1; i < len(path); i++ {
		nextPos := path[i]
		dx := nextPos.X - currentPos.X
		dy := nextPos.Y - currentPos.Y

		targetOrientation := -1 // Determine required orientation to move to next point

		if dx == 1 && dy == 0 {
			targetOrientation = OrientationEast
		} else if dx == -1 && dy == 0 {
			targetOrientation = OrientationWest
		} else if dx == 0 && dy == 1 { // Assuming Y increases downwards
			targetOrientation = OrientationSouth
		} else if dx == 0 && dy == -1 { // Assuming Y decreases upwards
			targetOrientation = OrientationNorth
		} else {
			// Path contains diagonal or invalid steps
			return nil, fmt.Errorf("invalid step in path: from %v to %v (non-cardinal or >1 step)", currentPos, nextPos)
		}

		// --- Calculate Turns Needed ---
		if currentOrientation != targetOrientation {
			// Calculate shortest turn direction (diff can be -3, -2, -1, 0, 1, 2, 3)
			diff := targetOrientation - currentOrientation
			// Handle wrap-around cases (e.g., West(3) to North(0) is diff=-3, should be Left(-1))
			if diff == 3 { // W(3) -> N(0)
				diff = -1 // Left turn
			} else if diff == -3 { // N(0) -> W(3)
				diff = 1 // Right turn
			}

			// Add turn commands based on difference
			if diff == 1 || diff == -3 { // Need to turn right
				commands = append(commands, CmdTurnRight)
			} else if diff == -1 || diff == 3 { // Need to turn left
				commands = append(commands, CmdTurnLeft)
			} else if diff == 2 || diff == -2 { // Need a 180-degree turn (two right turns)
				commands = append(commands, CmdTurnRight)
				commands = append(commands, CmdTurnRight)
			}
			// Update orientation after turning
			currentOrientation = targetOrientation
		}
		// --- End Turn Calculation ---

		// Add the forward command to move to the next cell
		commands = append(commands, CmdForward)
		currentPos = nextPos // Update position for the next iteration
	}

	return commands, nil
}

// --- Database Initialization ---
func initDB() {
	var err error
	// Check if DB file exists
	_, err = os.Stat(dbPath)
	isNewDB := os.IsNotExist(err)

	// Open connection (creates file if it doesn't exist)
	db, err = sql.Open("sqlite3", dbPath+"?_foreign_keys=on") // Enable foreign keys if needed later
	if err != nil {
		log.Fatalf("FATAL: Error opening database '%s': %v", dbPath, err)
	}

	// Optimize SQLite for concurrency (Write-Ahead Logging)
	_, err = db.Exec("PRAGMA journal_mode=WAL;")
	if err != nil {
		log.Printf("Warning: Failed to set journal_mode=WAL: %v", err)
	}
	// Set busy timeout to wait briefly if DB is locked
	_, err = db.Exec("PRAGMA busy_timeout = 5000;") // 5 seconds
	if err != nil {
		log.Printf("Warning: Failed to set busy_timeout: %v", err)
	}


	if isNewDB {
		log.Printf("Database file '%s' not found. Creating new schema...", dbPath)
		// Define the SQL schema
		schema := `
		CREATE TABLE tasks (
			id INTEGER PRIMARY KEY AUTOINCREMENT,
			station INTEGER NOT NULL,
			status TEXT NOT NULL DEFAULT 'pending' CHECK(status IN ('pending', 'assigned', 'completed', 'failed')),
			assigned_robot_id TEXT, -- Optional identifier for the robot
			created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
			updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
		);

		-- Index for efficiently finding pending tasks
		CREATE INDEX idx_tasks_status_id ON tasks (status, id);

		-- Trigger to automatically update the updated_at timestamp on changes
		CREATE TRIGGER update_tasks_updated_at
		AFTER UPDATE ON tasks FOR EACH ROW WHEN NEW.updated_at = OLD.updated_at -- Avoid infinite loop if updated_at is manually set
		BEGIN
			UPDATE tasks SET updated_at = CURRENT_TIMESTAMP WHERE id = OLD.id;
		END;
		`
		// Execute schema creation
		_, err = db.Exec(schema)
		if err != nil {
			db.Close() // Close DB before fatal exit
			log.Fatalf("FATAL: Error creating database schema: %v", err)
		}
		log.Println("Database schema created successfully.")
	} else {
		log.Printf("Existing database found at '%s'.", dbPath)
		// Optional: Add schema migration logic here if needed for existing DBs
	}

	// Ping DB to ensure connection is valid
	if err := db.Ping(); err != nil {
		db.Close()
		log.Fatalf("FATAL: Could not establish database connection: %v", err)
	}
	log.Println("Database connection established.")
}