from pathlib import Path

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from app.routers import map_router, nav_router

app = FastAPI(
    title="Indoor Navigation System",
    version="1.0.0",
    description="WiFi RSSI trilateration + EKF + Dijkstra indoor positioning",
)

# Allow browser requests when the frontend is opened as a local file
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── API routes (registered before static mount so they take priority) ─────────
app.include_router(map_router.router, prefix="/api/map", tags=["map"])
app.include_router(nav_router.router, prefix="/api/nav", tags=["navigation"])

# ── Static frontend files ─────────────────────────────────────────────────────
_FRONTEND = Path(__file__).parent / "frontend"

app.mount("/css",  StaticFiles(directory=_FRONTEND / "css"),                name="css")
app.mount("/js",   StaticFiles(directory=_FRONTEND / "js"),                 name="js")
app.mount("/maps", StaticFiles(directory=Path(__file__).parent / "maps"),   name="maps")


@app.get("/", include_in_schema=False)
async def index():
    return FileResponse(_FRONTEND / "index.html")


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
