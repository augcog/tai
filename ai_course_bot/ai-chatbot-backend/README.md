## Setup Instructions
1. install requirements `pip install -r requirements.txt`
2. configure env key, see `.env.example`
3. run server `python main.py`

### Configure SQLite Support
1. To enable SQLite support, please set `SQLDB = True` in `ai_course_bot/ai-chatbot-backend/app/core/actions/llama_seletor.py`

2. Download the appropriate `vector0.dylib` and `vss0.dylib` for your machine's hardware from https://github.com/asg017/sqlite-vss/releases. 

3. Place `vector0.dylib` and `vss0.dylib` files into the `rag/file_conversion_router/embedding/dist/debug` directory.