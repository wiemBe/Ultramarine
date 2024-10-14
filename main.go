package main

import (
	"database/sql"
	"log"

	"github.com/gin-gonic/gin"
	_ "github.com/mattn/go-sqlite3"
)

type Station struct {
	ID     int `json:"id"`
	Number int `json:"station"`
}

var db *sql.DB

func initDB() {
	var err error
	db, err = sql.Open("sqlite3", "./queue.db")
	if err != nil {
		log.Fatal(err)
	}

	// Create table if not exists
	_, err = db.Exec(`CREATE TABLE IF NOT EXISTS queue (
		id INTEGER PRIMARY KEY AUTOINCREMENT,
		station INTEGER
	)`)
	if err != nil {
		log.Fatal(err)
	}
}

func main() {
	initDB()
	defer db.Close()

	server := gin.Default()
	server.LoadHTMLGlob("templates/*")

	server.POST("/que", func(ctx *gin.Context) {
		var station Station
		if err := ctx.ShouldBindJSON(&station); err != nil {
			ctx.JSON(400, gin.H{"error": "Invalid input"})
			return
		}

		// Count current queue size
		var count int
		err := db.QueryRow("SELECT COUNT(*) FROM queue").Scan(&count)
		if err != nil {
			ctx.JSON(500, gin.H{"error": "Database error"})
			return
		}

		if count >= 5 {
			ctx.JSON(400, gin.H{"onQue": "0", "message": "Queue is full"})
			return
		}

		// Insert new station into queue
		_, err = db.Exec("INSERT INTO queue (station) VALUES (?)", station.Number)
		if err != nil {
			ctx.JSON(500, gin.H{"error": "Database error"})
			return
		}

		ctx.JSON(200, gin.H{
			"onQue": "1",
		})
	})

	server.GET("/queue", func(ctx *gin.Context) {
		rows, err := db.Query("SELECT id, station FROM queue ORDER BY id")
		if err != nil {
			ctx.JSON(500, gin.H{"error": "Database error"})
			return
		}
		defer rows.Close()

		var queue []Station
		for rows.Next() {
			var station Station
			if err := rows.Scan(&station.ID, &station.Number); err != nil {
				ctx.JSON(500, gin.H{"error": "Database error"})
				return
			}
			queue = append(queue, station)
		}

		ctx.JSON(200, gin.H{
			"queue": queue,
		})
	})

	server.GET("/available", func(ctx *gin.Context) {
		var nextStation Station
		err := db.QueryRow("SELECT id, station FROM queue ORDER BY id LIMIT 1").Scan(&nextStation.ID, &nextStation.Number)
		if err == sql.ErrNoRows {
			ctx.JSON(404, gin.H{
				"station": "Queue is empty",
			})
			return
		} else if err != nil {
			ctx.JSON(500, gin.H{"error": "Database error"})
			return
		}

		// Remove the first item from the queue
		_, err = db.Exec("DELETE FROM queue WHERE id = ?", nextStation.ID)
		if err != nil {
			ctx.JSON(500, gin.H{"error": "Database error"})
			return
		}

		ctx.JSON(200, gin.H{
			"station": nextStation.Number,
		})
	})

	server.GET("/", func(ctx *gin.Context) {
		ctx.HTML(200, "index.html", gin.H{
			"Title":   "UltraMarines",
			"Message": "this is message rendered",
		})
	})

	server.Run(":8080")
}