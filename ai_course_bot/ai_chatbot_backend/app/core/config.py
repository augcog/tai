from pydantic import BaseSettings


class Settings(BaseSettings):
    app_name: str = "My FastAPI App"
    log_level: str = "INFO"

    class Config:
        env_file = ".env"


settings = Settings()
