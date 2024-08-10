import os
from flask import Flask, request, jsonify, send_from_directory
from werkzeug.utils import secure_filename
import jwt
from datetime import datetime, timedelta
from functools import wraps
from flask_cors import CORS
from dotenv import load_dotenv

app = Flask(__name__)
cors = CORS(app, resources={
            r"/api/*": {"origins": "http://128.32.189.64:3000"}})
load_dotenv()

SECRET_KEY = os.getenv("SECURE_JWT")

BASE_DIR = '/Files'
app.config['UPLOAD_FOLDER'] = BASE_DIR
ALLOWED_EXTENSIONS = {'txt', 'pdf', 'png', 'jpg', 'jpeg', 'gif', 'zip'}


def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


def token_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        token = request.headers.get('Authorization')
        if not token:
            return jsonify({'message': 'Token is missing!'}), 403
        try:
            token = token.split(" ")[1]
            data = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
            if data['email'].split('@')[1] != 'berkeley.edu':
                return jsonify({'message': 'Invalid email domain!'}), 403
            if data['role'] != 'professor':
                return jsonify({'message': 'Access denied: only professors can perform this action!'}), 403
        except Exception as e:
            return jsonify({'message': str(e)}), 403
        return f(*args, **kwargs)
    return decorated


@app.route('/upload', methods=['POST'])
@token_required
def upload_file():
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'}), 400
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No selected file'}), 400
    if file and allowed_file(file.filename):
        filename = secure_filename(file.filename)
        file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
        return jsonify({'message': 'File uploaded successfully'}), 201


@app.route('/files', methods=['GET'])
@token_required
def list_files():
    files = os.listdir(app.config['UPLOAD_FOLDER'])
    return jsonify(files)


@app.route('/download/<filename>', methods=['GET'])
@token_required
def download_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)


@app.route('/delete/<filename>', methods=['DELETE'])
@token_required
def delete_file(filename):
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    if os.path.exists(file_path):
        os.remove(file_path)
        return jsonify({'message': 'File deleted successfully'}), 200
    else:
        return jsonify({'error': 'File not found'}), 404


@app.route('/update/<filename>', methods=['PUT'])
@token_required
def update_file(filename):
    if 'file' not in request.files:
        return jsonify({'error': 'No file part'}), 400
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No selected file'}), 400
    if file and allowed_file(file.filename):
        file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        if os.path.exists(file_path):
            os.remove(file_path)
        file.save(file_path)
        return jsonify({'message': 'File updated successfully'}), 200
    else:
        return jsonify({'error': 'Invalid file type'}), 400


if __name__ == '__main__':
    if not os.path.exists(BASE_DIR):
        os.makedirs(BASE_DIR)
    app.run(host='0.0.0.0', port=5000)
