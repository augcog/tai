## Setup Instructions
1. install requirements `pip install -r requirements.txt`
2. configure env key, see `.env.example`
3. run server `python main.py`

### Configure SQLite Support
1. Download the appropriate `vector0.dylib` and `vss0.dylib` for your machine's hardware from https://github.com/asg017/sqlite-vss/releases. Place `vector0.dylib` and `vss0.dylib` files into the `ai_course_bot/ai-chatbot-backend/app/core/actions/dist/debug` directory.

2. To enable SQLite support, please set `SQLDB = True` in `ai_course_bot/ai-chatbot-backend/app/core/actions/llama_seletor.py`

3. Make sure `current_dir` under `func llama_selector` is set to the correct file path

4. Running python ai_course_bot/ai-chatbot-backend/main.py and launch http://0.0.0.0:8000 in prefered browser