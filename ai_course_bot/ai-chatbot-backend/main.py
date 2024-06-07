import uvicorn
from fastapi import FastAPI
from starlette.responses import RedirectResponse
from app.v1.openai_mock import router as openapi_mock_router

import logging
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] {%(filename)s:%(funcName)s:%(lineno)d} %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("logs.log"), logging.StreamHandler()],
)

app = FastAPI()

app.include_router(openapi_mock_router)

@app.get("/")
def root():
    response = RedirectResponse(url='/docs')
    return response

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False)