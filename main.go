package main

import (
	"container/heap"
	"database/sql"
	"encoding/csv" // For reading map file
	"errors"
	"fmt"
	"log"
	"math"
	"net/http"
	"os" // For file operations
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	_ "github.com/mattn/go-sqlite3" // SQLite driver
)

// --- Configuration ---
const (
	dbPath         = "./robot_tasks.db"       // Database file name
	mapFilename    = "map.csv"          // Map file name (CSV from Tiled) - MAKE SURE THIS MATCHES YOUR FILE
	maxActiveTasks = 5                        // Max tasks in pending or assigned state
	serverPort     = ":8080"                  // Port for the web server
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
	CmdForward     = "FORWARD(1)"    // Move forward one grid unit (adjust unit distance in Arduino)
	CmdTurnLeft    = "TURN_LEFT(90)" // Turn 90 degrees left
	CmdTurnRight   = "TURN_RIGHT(90)" // Turn 90 degrees right
	CmdNone        = "NO_COMMAND"     // No command currently available (e.g., waiting for init)
	CmdComplete    = "PATH_COMPLETE"  // Indicates successful path completion
	CmdError       = "ERROR"          // Indicates a server-side error occurred (e.g., pathfinding failed)
)

// --- Robot Status Constants (Received from Robot) ---
const (
	StatusInit     = "INIT"     // Robot reporting initial state for the task
	StatusDone     = "DONE"     // Robot finished the last command successfully
	StatusObstacle = "OBSTACLE" // Robot encountered an unexpected obstacle
)


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

// RobotTaskState tracks the live state of an assigned task
type RobotTaskState struct {
	TaskID              int64
	TargetStation       int
	TargetPoint         Point    // Target X,Y coordinates
	CurrentPosition     Point    // Last reported X,Y position
	CurrentOrientation  int      // Last reported orientation
	PathCommands        []string // Sequence of commands
	CurrentCommandIndex int      // Index of the next command
	NeedsPathRecalc     bool     // Flag if path needs recalculation
	IsInitialized       bool     // Flag set after receiving INIT status
	Mutex               sync.Mutex // Protects concurrent access
}

// Structure for the API response of the map state endpoint
type RobotInfo struct {
	TaskID    int64 `json:"task_id"`
	X         int   `json:"x"`
	Y         int   `json:"y"`
	Theta     int   `json:"theta"` // Current Orientation (0=N, 1=E, 2=S, 3=W)
	Status    string `json:"status"` // Status from DB (assigned, etc.)
}

type MapStateResponse struct {
	Map struct {
		Width  int     `json:"width"`
		Height int     `json:"height"`
		Grid   [][]int `json:"grid"` // The actual world map
	} `json:"map"`
	Robots       []RobotInfo       `json:"robots"`       // List of active robot positions
	StationLocs map[string]Point `json:"station_locs"` // Map Station ID (string) to Point
}


// --- Global State ---
var db *sql.DB // Database connection handle

// Map data loaded from file
var worldMap [][]int // Populated by loadMapFromCSV
var mapWidth int     // Populated by loadMapFromCSV
var mapHeight int    // Populated by loadMapFromCSV

// Station ID to Map Coordinate mapping
// IMPORTANT: Ensure these coordinates are valid (0 or 2) in your loaded map!
var stationCoords = map[int]Point{
	1: {X: 5, Y: 0},  // Example: Station 1 - CHECK YOUR MAP!
	2: {X: 10, Y: 0},  // Example: Station 2 - CHECK YOUR MAP!
	3: {X: 15, Y: 0},  // Example: Station 3 - CHECK YOUR MAP!
	4: {X: 50, Y: 50},  // Example: Station 4 - CHECK YOUR MAP!
	// <<<--- ADD/MODIFY YOUR ACTUAL STATION COORDINATES HERE --->>>
}


// Map: Task ID -> *RobotTaskState (stores active path/command state)
var robotStates = make(map[int64]*RobotTaskState)
var robotStatesMutex sync.RWMutex // Mutex for the robotStates map itself

// --- Main Application ---
func main() {
	// --- Load Map Data First ---
	loadedMap, width, height, err := loadMapFromCSV(mapFilename)
	if err != nil {
		log.Fatalf("FATAL: Could not load map file '%s': %v", mapFilename, err)
	}
	worldMap = loadedMap
	mapWidth = width
	mapHeight = height
	// --- End Map Loading ---

	// Initialize Database
	initDB()
	defer db.Close()

	// Initialize robot state tracking map
	robotStates = make(map[int64]*RobotTaskState)

	// Setup Gin web server
	gin.SetMode(gin.ReleaseMode) // Use gin.DebugMode for more logs
	server := gin.Default()
	server.LoadHTMLGlob("templates/*") // Load HTML templates if needed

	// API Route Definitions
	tasksGroup := server.Group("/tasks")
	{
		tasksGroup.POST("", handleAddTask)
		tasksGroup.GET("", handleGetTasks)           // For JS to update queue list
		tasksGroup.POST("/next", handleGetNextTask)
		taskIDGroup := tasksGroup.Group("/:id")
		{
			taskIDGroup.POST("/complete", handleCompleteTask)
			taskIDGroup.POST("/fail", handleFailTask)
			taskIDGroup.POST("/report_status", handleReportStatus)
			taskIDGroup.GET("/next_command", handleGetNextCommand)
		}
	}

    // Map Visualization Route
	server.GET("/map/state", handleGetMapState)

	// Serve Frontend
	server.GET("/", func(ctx *gin.Context) {
		ctx.HTML(http.StatusOK, "index.html", gin.H{"Title": "Robot Task Queue & Map"}) // Updated title
	})

	log.Printf("Server starting on port %s...", serverPort)
	if err := server.Run(serverPort); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}

// --- Map Loading Function ---
// loadMapFromCSV reads a CSV file exported from Tiled and populates the map grid.
// Expects Tiled CSV export where: 0=Empty (Free), 1=Obstacle Tile GID, 2=Free Tile GID
func loadMapFromCSV(filename string) (gridMap [][]int, width int, height int, err error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, 0, 0, fmt.Errorf("error opening map file '%s': %w", filename, err)
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll() // Reads entire file
	if err != nil {
		return nil, 0, 0, fmt.Errorf("error reading map csv '%s': %w", filename, err)
	}

	if len(records) == 0 {
		return nil, 0, 0, fmt.Errorf("map file '%s' is empty", filename)
	}

	height = len(records)
	if height == 0 {
		return nil, 0, 0, fmt.Errorf("map file '%s' contains no rows", filename)
	}
	width = len(records[0])
	if width == 0 {
		return nil, 0, 0, fmt.Errorf("map file '%s' contains no columns in the first row", filename)
	}

	gridMap = make([][]int, height)
	log.Printf("Loading map '%s' (%dx%d)...", filename, width, height)

	for y, row := range records {
		gridMap[y] = make([]int, width)
		if len(row) != width {
			return nil, 0, 0, fmt.Errorf("map file '%s' has inconsistent row length at row %d (expected %d, got %d)", filename, y, width, len(row))
		}
		for x, cellStr := range row {
			trimmedStr := strings.TrimSpace(cellStr)
			if trimmedStr == "" {
				gridMap[y][x] = CellFree
				continue
			}

			tiledGid, convErr := strconv.Atoi(trimmedStr)
			if convErr != nil {
				return nil, 0, 0, fmt.Errorf("invalid non-numeric cell value '%s' at (%d,%d) in '%s': %w", cellStr, x, y, filename, convErr)
			}

			switch tiledGid {
			case 0: gridMap[y][x] = CellFree
			case 1: gridMap[y][x] = CellObstacle
			case 2: gridMap[y][x] = CellFree
			default:
				log.Printf("Warning: Unknown Tiled GID %d at (%d,%d) in '%s'. Treating as OBSTACLE.", tiledGid, x, y, filename)
				gridMap[y][x] = CellObstacle
			}
		}
	}
	log.Printf("Successfully loaded %d x %d map from %s", width, height, filename)
	return gridMap, width, height, nil
}


// --- Route Handlers ---

// handleAddTask: Adds a new task to the queue
func handleAddTask(ctx *gin.Context) {
	var input struct {
		Station int `json:"station" binding:"required"`
	}
	if err := ctx.ShouldBindJSON(&input); err != nil {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid input", "details": err.Error()})
		return
	}
	targetPoint, ok := stationCoords[input.Station]
	if !ok {
		ctx.JSON(http.StatusBadRequest, gin.H{"error": fmt.Sprintf("Invalid station ID: %d. Not defined.", input.Station)})
		return
	}
    if !isValid(targetPoint) { // Check if station coord is valid on map
        ctx.JSON(http.StatusBadRequest, gin.H{"error": fmt.Sprintf("Configuration Error: Station %d maps to an invalid location (%d,%d) on the map.", input.Station, targetPoint.X, targetPoint.Y)})
        return
    }

	var activeCount int
	err := db.QueryRow("SELECT COUNT(*) FROM tasks WHERE status = ? OR status = ?", StatusPending, StatusAssigned).Scan(&activeCount)
	if err != nil { log.Printf("Error counting active tasks: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error":"db error"}); return }
	if activeCount >= maxActiveTasks {
		ctx.JSON(http.StatusServiceUnavailable, gin.H{"error": "Queue is full"})
		return
	}

	stmt, err := db.Prepare("INSERT INTO tasks (station, status) VALUES (?, ?)")
	if err != nil { log.Printf("Error preparing insert: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error":"db error"}); return }
	defer stmt.Close()
	res, err := stmt.Exec(input.Station, StatusPending)
	if err != nil { log.Printf("Error inserting task: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error":"db error"}); return }
	newID, _ := res.LastInsertId()

	var createdTask Task
	err = db.QueryRow(`SELECT id, station, status, assigned_robot_id, created_at, updated_at FROM tasks WHERE id = ?`, newID).Scan(
		&createdTask.ID, &createdTask.Station, &createdTask.Status, &createdTask.AssignedRobotID, &createdTask.CreatedAt, &createdTask.UpdatedAt,
	)
	if err != nil { log.Printf("Error fetching created task %d: %v", newID, err); ctx.JSON(http.StatusCreated, gin.H{"message": "Task created successfully", "id": newID}); return } // Log error, but proceed
	ctx.JSON(http.StatusCreated, createdTask)
}

// handleGetTasks: Retrieves a list of tasks.
func handleGetTasks(ctx *gin.Context) {
	rows, err := db.Query(`SELECT id, station, status, assigned_robot_id, created_at, updated_at FROM tasks ORDER BY created_at DESC`)
	if err != nil { log.Printf("Error querying tasks: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return }
	defer rows.Close()
	var tasks []Task
	for rows.Next() {
		var task Task
		var assignedRobotID sql.NullString
		if err := rows.Scan(&task.ID, &task.Station, &task.Status, &assignedRobotID, &task.CreatedAt, &task.UpdatedAt); err != nil {
			log.Printf("Error scanning task row: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return
		}
		if assignedRobotID.Valid { task.AssignedRobotID = &assignedRobotID.String }
		tasks = append(tasks, task)
	}
	if err := rows.Err(); err != nil { log.Printf("Error after task rows: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return }
	ctx.JSON(http.StatusOK, gin.H{"tasks": tasks})
}

// handleGetNextTask: Robot requests a task; server assigns the oldest pending one.
func handleGetNextTask(ctx *gin.Context) {
	tx, err := db.Begin(); if err != nil { log.Printf("Error begin tx: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return }; defer tx.Rollback()
	var nextTask Task; var assignedRobotID sql.NullString
	err = tx.QueryRow(`SELECT id, station, status, assigned_robot_id, created_at, updated_at FROM tasks WHERE status = ? ORDER BY id LIMIT 1`, StatusPending).Scan(
		&nextTask.ID, &nextTask.Station, &nextTask.Status, &assignedRobotID, &nextTask.CreatedAt, &nextTask.UpdatedAt,
	)
	if err == sql.ErrNoRows { ctx.JSON(http.StatusNotFound, gin.H{"message": "No pending tasks available"}); return }
	if err != nil { log.Printf("Error find next task: %v", err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return }
	_, err = tx.Exec("UPDATE tasks SET status = ? WHERE id = ?", StatusAssigned, nextTask.ID)
	if err != nil { log.Printf("Error assign task %d: %v", nextTask.ID, err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return }
	targetPoint, ok := stationCoords[nextTask.Station]; if !ok { log.Printf("Config Err: Sta %d miss", nextTask.Station); tx.Rollback(); ctx.JSON(http.StatusInternalServerError, gin.H{"error":"config error"}); return }
    if !isValid(targetPoint) { log.Printf("Config Err: Sta %d invalid map loc", nextTask.Station); tx.Rollback(); ctx.JSON(http.StatusInternalServerError, gin.H{"error":"config error"}); return }

	newState := &RobotTaskState{ TaskID: nextTask.ID, TargetStation: nextTask.Station, TargetPoint: targetPoint, CurrentCommandIndex: 0, PathCommands: nil, NeedsPathRecalc: true, IsInitialized: false }
	robotStatesMutex.Lock(); if _, exists := robotStates[nextTask.ID]; exists { log.Printf("Warn: Overwriting state Task %d", nextTask.ID) }; robotStates[nextTask.ID] = newState; robotStatesMutex.Unlock()
	if err := tx.Commit(); err != nil {
		log.Printf("Error commit assign %d: %v", nextTask.ID, err); robotStatesMutex.Lock(); delete(robotStates, nextTask.ID); robotStatesMutex.Unlock(); ctx.JSON(http.StatusInternalServerError, gin.H{"error": "db error"}); return
	}
	log.Printf("Assigned Task ID %d (Station %d)\n", nextTask.ID, nextTask.Station); nextTask.Status = StatusAssigned
	ctx.JSON(http.StatusOK, nextTask)
}

// handleReportStatus: Robot reports its state.
func handleReportStatus(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64); if err != nil { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID"}); return }
	var input struct { X int `json:"x"`; Y int `json:"y"`; Theta int `json:"theta"`; LastCmdStatus string `json:"last_cmd_status" binding:"required"`}
	if err := ctx.ShouldBindJSON(&input); err != nil { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid input", "details": err.Error()}); return }
	if input.Theta < OrientationNorth || input.Theta > OrientationWest { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid theta"}); return }
	inputStatusUpper := strings.ToUpper(input.LastCmdStatus)
	if !(inputStatusUpper == StatusInit || inputStatusUpper == StatusDone || inputStatusUpper == StatusObstacle) { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid last_cmd_status"}); return }
	if input.X < 0 || input.X >= mapWidth || input.Y < 0 || input.Y >= mapHeight { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Coords out of bounds"}); return }

	robotStatesMutex.RLock(); state, exists := robotStates[taskID]; robotStatesMutex.RUnlock()
	if !exists { log.Printf("Status report inactive Task %d", taskID); ctx.JSON(http.StatusNotFound, gin.H{"error": "Task state not found"}); return }

	state.Mutex.Lock(); defer state.Mutex.Unlock()
	// log.Printf("Status Report Task %d: Pos=(%d,%d), Theta=%d, LastCmd=%s", taskID, input.X, input.Y, input.Theta, inputStatusUpper) // Reduce log verbosity
	state.CurrentPosition = Point{X: input.X, Y: input.Y}; state.CurrentOrientation = input.Theta

	switch inputStatusUpper {
	case StatusInit: state.IsInitialized = true; state.NeedsPathRecalc = true; state.CurrentCommandIndex = 0
	case StatusDone:
		if state.PathCommands == nil { state.NeedsPathRecalc = true } else if state.CurrentCommandIndex < len(state.PathCommands) { state.CurrentCommandIndex++ }
		if !state.NeedsPathRecalc { state.NeedsPathRecalc = false }
	case StatusObstacle: state.NeedsPathRecalc = true; state.PathCommands = nil; state.CurrentCommandIndex = 0; log.Printf("Obstacle Task %d at (%d,%d)", taskID, input.X, input.Y)
	}
	ctx.JSON(http.StatusOK, gin.H{"message": "Status received"})
}

// handleGetNextCommand: Robot requests the next navigation command.
func handleGetNextCommand(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64); if err != nil { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID"}); return }
	robotStatesMutex.RLock(); state, exists := robotStates[taskID]; robotStatesMutex.RUnlock()
	if !exists { ctx.JSON(http.StatusNotFound, gin.H{"command": CmdError, "error": "Task state not found"}); return }

	state.Mutex.Lock(); defer state.Mutex.Unlock()
	if !state.IsInitialized { ctx.JSON(http.StatusOK, gin.H{"command": CmdNone, "message": "Waiting for initial robot report"}); return }

	if state.PathCommands == nil || state.NeedsPathRecalc {
		log.Printf("Calc path Task %d: (%d,%d)@%d -> Sta %d (%d,%d)", taskID, state.CurrentPosition.X, state.CurrentPosition.Y, state.CurrentOrientation, state.TargetStation, state.TargetPoint.X, state.TargetPoint.Y)
		pathPoints, pathErr := findPathAStar(state.CurrentPosition, state.TargetPoint)
		if pathErr != nil { log.Printf("Pathfind Err Task %d: %v", taskID, pathErr); ctx.JSON(http.StatusOK, gin.H{"command": CmdError, "error": fmt.Sprintf("Pathfind fail: %v", pathErr)}); return }
		if len(pathPoints) <= 1 {
			if state.CurrentPosition == state.TargetPoint { go internalCompleteTask(taskID); ctx.JSON(http.StatusOK, gin.H{"command": CmdComplete}); return }
			log.Printf("A* Err Task %d: Empty/1pt path", taskID); ctx.JSON(http.StatusOK, gin.H{"command": CmdError, "error": "Path not found/unreachable"}); return
		}
		commands, cmdErr := pathToCommands(pathPoints, state.CurrentOrientation)
		if cmdErr != nil { log.Printf("Cmd Gen Err Task %d: %v", taskID, cmdErr); ctx.JSON(http.StatusOK, gin.H{"command": CmdError, "error": fmt.Sprintf("Cmd gen fail: %v", cmdErr)}); return }
		log.Printf("Task %d: Gen %d commands.", taskID, len(commands)); state.PathCommands = commands; state.CurrentCommandIndex = 0; state.NeedsPathRecalc = false
	}

	if state.PathCommands == nil { log.Printf("Internal Err Task %d: Nil PathCmds", taskID); ctx.JSON(http.StatusInternalServerError, gin.H{"command": CmdError, "error": "Internal path error"}); return }
	if state.CurrentCommandIndex < len(state.PathCommands) {
		nextCmd := state.PathCommands[state.CurrentCommandIndex]
		// log.Printf("Task %d: Send Cmd %d/%d: %s", taskID, state.CurrentCommandIndex+1, len(state.PathCommands), nextCmd); // Reduce verbosity
		ctx.JSON(http.StatusOK, gin.H{"command": nextCmd})
	} else {
		log.Printf("Task %d: Path finished.", taskID); go internalCompleteTask(taskID); ctx.JSON(http.StatusOK, gin.H{"command": CmdComplete})
	}
}

// handleGetMapState: Returns map grid, robot positions, and station locations.
func handleGetMapState(ctx *gin.Context) {
	var response MapStateResponse
	response.Map.Width = mapWidth; response.Map.Height = mapHeight; response.Map.Grid = worldMap
	response.StationLocs = make(map[string]Point); for id, point := range stationCoords { response.StationLocs[strconv.Itoa(id)] = point }
	response.Robots = make([]RobotInfo, 0)

	robotStatesMutex.RLock(); activeTaskIDs := make([]int64, 0, len(robotStates)); for taskID := range robotStates { activeTaskIDs = append(activeTaskIDs, taskID) }; robotStatesMutex.RUnlock()

	for _, taskID := range activeTaskIDs {
		robotStatesMutex.RLock(); state, exists := robotStates[taskID]; robotStatesMutex.RUnlock()
		if !exists { continue }
		state.Mutex.Lock() // Lock individual state
		if state.IsInitialized {
			var currentDBStatus string; err := db.QueryRow("SELECT status FROM tasks WHERE id = ?", taskID).Scan(&currentDBStatus); if err != nil { currentDBStatus = "db_err" }
			robotInfo := RobotInfo{ TaskID: state.TaskID, X: state.CurrentPosition.X, Y: state.CurrentPosition.Y, Theta: state.CurrentOrientation, Status: currentDBStatus }
			response.Robots = append(response.Robots, robotInfo)
		}
		state.Mutex.Unlock() // Unlock individual state
	}
	ctx.JSON(http.StatusOK, response)
}

// handleCompleteTask: Finalizes task completion.
func handleCompleteTask(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64); if err != nil { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID"}); return }
	if err := internalCompleteTask(taskID); err != nil {
		if strings.Contains(err.Error(), "Task state not found") { ctx.JSON(http.StatusOK, gin.H{"message": "Task already complete/cleared", "id": taskID}) } else { log.Printf("Err final complete %d: %v", taskID, err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()}) }
		return
	}
	ctx.JSON(http.StatusOK, gin.H{"message": "Task marked as completed", "id": taskID})
}

// handleFailTask: Finalizes task failure.
func handleFailTask(ctx *gin.Context) {
	taskID, err := strconv.ParseInt(ctx.Param("id"), 10, 64); if err != nil { ctx.JSON(http.StatusBadRequest, gin.H{"error": "Invalid task ID"}); return }
	if err := internalFailTask(taskID); err != nil {
		if strings.Contains(err.Error(), "Task state not found") { ctx.JSON(http.StatusOK, gin.H{"message": "Task already failed/cleared", "id": taskID}) } else { log.Printf("Err final fail %d: %v", taskID, err); ctx.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()}) }
		return
	}
	ctx.JSON(http.StatusOK, gin.H{"message": "Task marked as failed", "id": taskID})
}


// --- Internal Helper Functions ---

func internalCompleteTask(taskID int64) error {
	// log.Printf("Internal complete Task %d...", taskID) // Reduce verbosity
	robotStatesMutex.Lock(); _, exists := robotStates[taskID]; if exists { delete(robotStates, taskID); log.Printf("State: Cleaned complete %d", taskID) } else { log.Printf("State: Not found complete %d", taskID) }; robotStatesMutex.Unlock()
	res, err := db.Exec("UPDATE tasks SET status = ? WHERE id = ? AND status != ?", StatusCompleted, taskID, StatusCompleted)
	if err != nil { log.Printf("DB Err complete %d: %v", taskID, err); return fmt.Errorf("db error: %w", err)}
	rowsAffected, _ := res.RowsAffected(); if rowsAffected > 0 { log.Printf("DB: Marked complete %d", taskID) }
	return nil
 }
func internalFailTask(taskID int64) error {
	// log.Printf("Internal fail Task %d...", taskID) // Reduce verbosity
	robotStatesMutex.Lock(); _, exists := robotStates[taskID]; if exists { delete(robotStates, taskID); log.Printf("State: Cleaned fail %d", taskID) } else { log.Printf("State: Not found fail %d", taskID) }; robotStatesMutex.Unlock()
	res, err := db.Exec("UPDATE tasks SET status = ? WHERE id = ? AND status != ?", StatusFailed, taskID, StatusFailed)
	if err != nil { log.Printf("DB Err fail %d: %v", taskID, err); return fmt.Errorf("db error: %w", err)}
	rowsAffected, _ := res.RowsAffected(); if rowsAffected > 0 { log.Printf("DB: Marked fail %d", taskID) }
	return nil
 }

// --- A* Pathfinding Implementation ---

type aStarNode struct { p Point; g, h, f float64; parent *aStarNode; index int }
type priorityQueue []*aStarNode
func (pq priorityQueue) Len() int            { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool  { return pq[i].f < pq[j].f }
func (pq priorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i]; pq[i].index = i; pq[j].index = j }
func (pq *priorityQueue) Push(x interface{}) { n := len(*pq); node := x.(*aStarNode); node.index = n; *pq = append(*pq, node) }
func (pq *priorityQueue) Pop() interface{}   { old := *pq; n := len(old); node := old[n-1]; old[n-1] = nil; node.index = -1; *pq = old[0 : n-1]; return node }
func heuristic(a, b Point) float64           { return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y)) }
func isValid(p Point) bool { if p.X < 0 || p.X >= mapWidth || p.Y < 0 || p.Y >= mapHeight {	return false }; return worldMap[p.Y][p.X] == CellFree }
func getNeighbors(p Point) []Point { neighbors := []Point{}; moves := []Point{{X:0,Y:-1},{X:1,Y:0},{X:0,Y:1},{X:-1,Y:0}}; for _, move := range moves { neighbor := Point{X: p.X + move.X, Y: p.Y + move.Y}; if isValid(neighbor) { neighbors = append(neighbors, neighbor) } }; return neighbors }
func findPathAStar(start, goal Point) ([]Point, error) {
	if !isValid(start) { return nil, fmt.Errorf("start invalid (%d,%d)", start.X, start.Y) }
	if !isValid(goal) { return nil, fmt.Errorf("goal invalid (%d,%d)", goal.X, goal.Y) }
	if start == goal { return []Point{start}, nil }
	openSet := make(priorityQueue, 0); heap.Init(&openSet); closedSet := make(map[Point]bool); nodeMap := make(map[Point]*aStarNode)
	startNode := &aStarNode{p: start, g: 0, h: heuristic(start, goal)}; startNode.f = startNode.g + startNode.h
	heap.Push(&openSet, startNode); nodeMap[start] = startNode
	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*aStarNode)
		if current.p == goal { path := []Point{}; temp := current; for temp != nil { path = append(path, temp.p); temp = temp.parent }; for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 { path[i], path[j] = path[j], path[i] }; return path, nil }
		closedSet[current.p] = true
		for _, neighborPoint := range getNeighbors(current.p) {
			if closedSet[neighborPoint] { continue }
			gScore := current.g + 1.0; neighborNode, exists := nodeMap[neighborPoint]; isNewerBetter := false
			if !exists { neighborNode = &aStarNode{p: neighborPoint}; nodeMap[neighborPoint] = neighborNode; isNewerBetter = true } else if gScore < neighborNode.g { isNewerBetter = true }
			if isNewerBetter {
				neighborNode.parent = current; neighborNode.g = gScore; neighborNode.h = heuristic(neighborPoint, goal); neighborNode.f = neighborNode.g + neighborNode.h
				if !exists { heap.Push(&openSet, neighborNode) } else { heap.Fix(&openSet, neighborNode.index) }
			}
		}
	}
	return nil, errors.New("path not found")
}

// --- Command Generation ---
func pathToCommands(path []Point, startOrientation int) ([]string, error) {
	if len(path) < 2 { return []string{}, nil }
	commands := []string{}; currentOrientation := startOrientation; currentPos := path[0]
	for i := 1; i < len(path); i++ {
		nextPos := path[i]; dx := nextPos.X - currentPos.X; dy := nextPos.Y - currentPos.Y; targetOrientation := -1
		if dx == 1 && dy == 0 { targetOrientation = OrientationEast } else if dx == -1 && dy == 0 { targetOrientation = OrientationWest } else if dx == 0 && dy == 1 { targetOrientation = OrientationSouth } else if dx == 0 && dy == -1 { targetOrientation = OrientationNorth } else { return nil, fmt.Errorf("invalid step: %v to %v", currentPos, nextPos) }
		if currentOrientation != targetOrientation {
			diff := targetOrientation - currentOrientation; if diff == 3 { diff = -1 }; if diff == -3 { diff = 1 }
			if diff == 1 { commands = append(commands, CmdTurnRight) } else if diff == -1 { commands = append(commands, CmdTurnLeft) } else if diff == 2 || diff == -2 { commands = append(commands, CmdTurnRight); commands = append(commands, CmdTurnRight) }
			currentOrientation = targetOrientation
		}
		commands = append(commands, CmdForward); currentPos = nextPos
	}
	return commands, nil
}

// --- Database Initialization ---
func initDB() {
	var err error; _, err = os.Stat(dbPath); isNewDB := os.IsNotExist(err)
	db, err = sql.Open("sqlite3", dbPath+"?_foreign_keys=on"); if err != nil { log.Fatalf("FATAL: DB Open error: %v", err) }
    if err = db.Ping(); err != nil { db.Close(); log.Fatalf("FATAL: DB Ping error: %v", err) }
	_, err = db.Exec("PRAGMA journal_mode=WAL;"); if err != nil { log.Printf("Warn: WAL mode failed: %v", err) }
	_, err = db.Exec("PRAGMA busy_timeout = 5000;"); if err != nil { log.Printf("Warn: Busy timeout failed: %v", err) }
	if isNewDB {
		log.Printf("DB '%s' not found. Creating schema...", dbPath)
		schema := `CREATE TABLE tasks (id INTEGER PRIMARY KEY AUTOINCREMENT, station INTEGER NOT NULL, status TEXT NOT NULL DEFAULT 'pending' CHECK(status IN ('pending', 'assigned', 'completed', 'failed')), assigned_robot_id TEXT, created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP, updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP); CREATE INDEX idx_tasks_status_id ON tasks (status, id); CREATE TRIGGER update_tasks_updated_at AFTER UPDATE ON tasks FOR EACH ROW WHEN NEW.updated_at = OLD.updated_at BEGIN UPDATE tasks SET updated_at = CURRENT_TIMESTAMP WHERE id = OLD.id; END;`
		_, err = db.Exec(schema); if err != nil { db.Close(); log.Fatalf("FATAL: Schema creation error: %v", err) }
		log.Println("DB schema created.")
	} else { log.Printf("Existing DB found: '%s'.", dbPath) }
	log.Println("DB setup complete.")
}