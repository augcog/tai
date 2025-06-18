// Constants for API URLs and common values
const BASE_URL = "/v1/files";
const AUTH_TOKEN = "Bearer your_access_token_here";

// Store current files for reference
let currentFiles = [];

// After DOM content is loaded, initialize the application
document.addEventListener("DOMContentLoaded", () => {
    // Add event listeners to various forms and inputs
    document.getElementById("files-form").addEventListener("submit", handleFilesSubmit);
    document.getElementById("file-metadata-form").addEventListener("submit", handleFileMetadataSubmit);
    document.getElementById("file-download-form").addEventListener("submit", handleFileDownloadSubmit);
    document.getElementById("stats-form").addEventListener("submit", handleStatsSubmit);
    document.getElementById("clear-response-btn").addEventListener("click", clearResponse);
    
    // Initialize by listing files
    listFiles();
});

// Handler for listing files with filters
async function handleFilesSubmit(event) {
    event.preventDefault();
    
    const courseCode = document.getElementById("course-code-input").value.trim();
    const category = document.getElementById("category-input").value;
    const search = document.getElementById("search-input").value.trim();
    const page = parseInt(document.getElementById("page-input").value) || 1;
    const limit = parseInt(document.getElementById("limit-input").value) || 100;
    
    await listFiles(courseCode, category, search, page, limit);
}

// Handler for getting file metadata and content
async function handleFileMetadataSubmit(event) {
    event.preventDefault();
    const fileId = document.getElementById("file-id-input").value.trim();
    
    if (!fileId) {
        showError("Please enter a file UUID");
        return;
    }
    
    await getFileWithContent(fileId);
}

// Handler for downloading a file
async function handleFileDownloadSubmit(event) {
    event.preventDefault();
    const fileId = document.getElementById("download-file-id-input").value.trim();
    
    if (!fileId) {
        showError("Please enter a file UUID");
        return;
    }
    
    await downloadFile(fileId);
}

// Handler for getting file statistics
async function handleStatsSubmit(event) {
    event.preventDefault();
    await getFileStats();
}

// Function to list files with optional filters
async function listFiles(courseCode = "", category = "", search = "", page = 1, limit = 100) {
    showLoading();
    
    try {
        const params = new URLSearchParams();
        if (courseCode) params.append("course_code", courseCode);
        if (category) params.append("category", category);
        if (search) params.append("search", search);
        if (page > 1) params.append("page", page);
        if (limit !== 100) params.append("limit", limit);
        
        const queryString = params.toString();
        const url = queryString ? `${BASE_URL}?${queryString}` : BASE_URL;
        
        const response = await fetch(url, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN,
                "Accept": "application/json"
            }
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to list files");
        }
        
        // Log the data to console for debugging
        console.log("Files API Response:", data);
        
        // Store current files for reference
        currentFiles = data.files || [];
        
        // Update UI with the file list
        renderFileList(data);
        displayJsonResponse(data);
        
    } catch (error) {
        showError(`Error listing files: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get file metadata and content by UUID
async function getFileWithContent(fileId) {
    showLoading();
    
    try {
        if (!isValidUUID(fileId)) {
            throw new Error("Invalid UUID format");
        }
        
        const url = `${BASE_URL}/${fileId}`;
        
        const response = await fetch(url, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN,
                "Accept": "application/json"
            }
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to get file");
        }
        
        console.log("File with content:", data);
        
        // Update UI with file data (metadata + content)
        renderFilePreview(data);
        displayJsonResponse(data);
        
    } catch (error) {
        showError(`Error getting file: ${error.message}`);
        
        // Show error in preview panel too
        const previewElement = document.getElementById("preview-content");
        previewElement.innerHTML = `
            <div class="error-message">
                <i class="fas fa-exclamation-triangle"></i>
                <p>Error retrieving file: ${error.message}</p>
            </div>
        `;
    } finally {
        hideLoading();
    }
}

// Function to download a file by UUID
async function downloadFile(fileId) {
    showLoading();
    
    try {
        if (!isValidUUID(fileId)) {
            throw new Error("Invalid UUID format");
        }
        
        const url = `${BASE_URL}/${fileId}/download`;
        
        const response = await fetch(url, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN
            }
        });
        
        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || "Failed to download file");
        }
        
        // Get filename from Content-Disposition header or use default
        const contentDisposition = response.headers.get("Content-Disposition");
        let filename = "download";
        if (contentDisposition) {
            const match = contentDisposition.match(/filename="?([^"]+)"?/);
            if (match) filename = match[1];
        }
        
        // Get the file content as blob
        const blob = await response.blob();
        
        // Create download link
        const downloadUrl = window.URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.style.display = "none";
        a.href = downloadUrl;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(downloadUrl);
        document.body.removeChild(a);
        
        displayJsonResponse({
            message: "File downloaded successfully",
            filename: filename,
            size: blob.size,
            type: blob.type
        });
        
    } catch (error) {
        showError(`Error downloading file: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get file statistics
async function getFileStats() {
    showLoading();
    
    try {
        const url = `${BASE_URL}/stats/summary`;
        
        const response = await fetch(url, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN,
                "Accept": "application/json"
            }
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to get file statistics");
        }
        
        console.log("File statistics:", data);
        
        // Update UI with statistics
        renderFileStats(data);
        displayJsonResponse(data);
        
    } catch (error) {
        showError(`Error getting file statistics: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to render file list
function renderFileList(data) {
    const fileListElement = document.getElementById("file-list");
    
    if (!data.files || data.files.length === 0) {
        fileListElement.innerHTML = `
            <div class="initial-message">
                <i class="fas fa-folder-open"></i>
                <p>No files found</p>
            </div>
        `;
        return;
    }
    
    let html = `
        <div class="file-list-header">
            <h3>Files (${data.total_count} total, page ${data.page} of ${Math.ceil(data.total_count / data.limit)})</h3>
        </div>
        <div class="file-grid">
    `;
    
    data.files.forEach(file => {
        const icon = getFileIcon(file.filename);
        const sizeFormatted = formatFileSize(file.size_bytes || 0);
        
        html += `
            <div class="file-item" data-file-id="${file.uuid}" onclick="selectFile('${file.uuid}', '${file.filename}')">
                <div class="file-icon">
                    <i class="${icon}"></i>
                </div>
                <div class="file-info">
                    <div class="file-name" title="${file.filename}">${file.filename}</div>
                    <div class="file-details">
                        <span class="file-size">${sizeFormatted}</span>
                        ${file.course ? `<span class="file-course">${file.course}</span>` : ''}
                        ${file.category ? `<span class="file-category">${file.category}</span>` : ''}
                    </div>
                    <div class="file-uuid" title="File UUID">${file.uuid}</div>
                </div>
            </div>
        `;
    });
    
    html += `</div>`;
    
    // Add pagination if applicable
    if (data.has_prev || data.has_next) {
        html += `
            <div class="pagination">
                ${data.has_prev ? `<button onclick="changePage(${data.page - 1})">Previous</button>` : ''}
                <span>Page ${data.page}</span>
                ${data.has_next ? `<button onclick="changePage(${data.page + 1})">Next</button>` : ''}
            </div>
        `;
    }
    
    fileListElement.innerHTML = html;
}

// Function to render file statistics
function renderFileStats(data) {
    const previewElement = document.getElementById("preview-content");
    
    let html = `
        <div class="stats-container">
            <h3>File System Statistics</h3>
            <div class="stats-grid">
                <div class="stat-item">
                    <div class="stat-value">${data.total_files}</div>
                    <div class="stat-label">Total Files</div>
                </div>
    `;
    
    if (data.courses) {
        html += `
                <div class="stat-item">
                    <div class="stat-value">${Object.keys(data.courses).length}</div>
                    <div class="stat-label">Courses</div>
                </div>
        `;
        
        html += `
            </div>
            <div class="course-breakdown">
                <h4>Files by Course</h4>
                <div class="course-list">
        `;
        
        Object.entries(data.courses).forEach(([course, count]) => {
            html += `
                <div class="course-item">
                    <span class="course-name">${course}</span>
                    <span class="course-count">${count} files</span>
                </div>
            `;
        });
        
        html += `</div></div>`;
    } else {
        html += `</div>`;
    }
    
    if (data.last_updated) {
        html += `
            <div class="last-updated">
                <small>Last updated: ${new Date(data.last_updated).toLocaleString()}</small>
            </div>
        `;
    }
    
    html += `</div>`;
    
    previewElement.innerHTML = html;
}

// Function to render file preview/metadata with content
function renderFilePreview(fileData) {
    const previewElement = document.getElementById("preview-content");
    
    if (!fileData) {
        previewElement.innerHTML = `
            <div class="error-message">
                <i class="fas fa-exclamation-triangle"></i>
                <p>No file data available</p>
            </div>
        `;
        return;
    }
    
    const icon = getFileIcon(fileData.filename);
    const sizeFormatted = formatFileSize(fileData.size_bytes || 0);
    
    let html = `
        <div class="file-preview">
            <div class="file-header">
                <div class="file-icon-large">
                    <i class="${icon}"></i>
                </div>
                <div class="file-details-large">
                    <h3>${fileData.filename}</h3>
                    <p class="file-uuid">UUID: ${fileData.uuid}</p>
                    <p class="file-size">Size: ${sizeFormatted}</p>
                    <p class="file-mime">Type: ${fileData.mime_type}</p>
                    ${fileData.course ? `<p class="file-course">Course: ${fileData.course}</p>` : ''}
                    ${fileData.category ? `<p class="file-category">Category: ${fileData.category}</p>` : ''}
                    ${fileData.title ? `<p class="file-title">Title: ${fileData.title}</p>` : ''}
                </div>
            </div>
    `;
    
    if (fileData.created_at) {
        html += `<p class="file-created">Created: ${new Date(fileData.created_at).toLocaleString()}</p>`;
    }
    
    if (fileData.modified_at) {
        html += `<p class="file-modified">Modified: ${new Date(fileData.modified_at).toLocaleString()}</p>`;
    }
    
    // Render file content based on type
    if (fileData.content) {
        html += `<div class="content-section">`;
        
        if (fileData.mime_type.includes("image/")) {
            // Handle image files
            html += `
                <h4>Preview</h4>
                <div class="image-preview">
                    <img src="data:${fileData.mime_type};base64,${fileData.content}" alt="${fileData.filename}" />
                </div>
            `;
        } else if (fileData.mime_type.includes("text/") || fileData.mime_type.includes("application/json")) {
            // Handle text files
            try {
                const decodedContent = atob(fileData.content);
                html += `
                    <h4>Content Preview</h4>
                    <div class="text-preview">
                        <pre>${decodedContent.substring(0, 2000)}${decodedContent.length > 2000 ? '...' : ''}</pre>
                    </div>
                `;
            } catch (e) {
                html += `<p class="preview-error">Cannot preview text content</p>`;
            }
        } else if (fileData.mime_type.includes("application/pdf")) {
            // Handle PDF files
            html += `
                <h4>PDF Preview</h4>
                <p>PDF content available. Use download button to view the full document.</p>
            `;
        } else {
            // Handle other binary files
            html += `
                <h4>Binary File</h4>
                <p>This is a binary file (${fileData.mime_type}). Use download button to access content.</p>
            `;
        }
        
        html += `</div>`;
    }
    
    html += `
            <div class="file-actions">
                <button onclick="downloadFile('${fileData.uuid}')" class="btn">
                    <i class="fas fa-download"></i> Download
                </button>
            </div>
        </div>
    `;
    
    previewElement.innerHTML = html;
}

// Function to select a file and populate UUID inputs, and automatically get its content
function selectFile(fileId, fileName) {
    // Highlight selected file
    document.querySelectorAll('.file-item').forEach(item => {
        item.classList.remove('selected');
    });
    document.querySelector(`[data-file-id="${fileId}"]`).classList.add('selected');
    
    // Populate UUID inputs
    document.getElementById("file-id-input").value = fileId;
    document.getElementById("download-file-id-input").value = fileId;
    
    console.log(`Selected file: ${fileName} (${fileId})`);
    
    // Automatically get file content and metadata
    getFileWithContent(fileId);
}

// Function to change page
function changePage(page) {
    const courseCode = document.getElementById("course-code-input").value.trim();
    const category = document.getElementById("category-input").value;
    const search = document.getElementById("search-input").value.trim();
    const limit = parseInt(document.getElementById("limit-input").value) || 100;
    
    // Update page input
    document.getElementById("page-input").value = page;
    
    listFiles(courseCode, category, search, page, limit);
}

// Utility function to validate UUID format
function isValidUUID(uuid) {
    const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
    return uuidRegex.test(uuid);
}

// Function to display JSON response
function displayJsonResponse(data) {
    const responseElement = document.getElementById("json-response");
    responseElement.innerHTML = syntaxHighlight(JSON.stringify(data, null, 2));
}

// Function to clear response
function clearResponse() {
    document.getElementById("json-response").innerHTML = "";
}

// Function to add syntax highlighting to JSON
function syntaxHighlight(json) {
    json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    return json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
        var cls = 'number';
        if (/^"/.test(match)) {
            if (/:$/.test(match)) {
                cls = 'key';
            } else {
                cls = 'string';
            }
        } else if (/true|false/.test(match)) {
            cls = 'boolean';
        } else if (/null/.test(match)) {
            cls = 'null';
        }
        return '<span class="' + cls + '">' + match + '</span>';
    });
}

// Function to get appropriate icon for file type
function getFileIcon(fileName) {
    const extension = fileName.split('.').pop().toLowerCase();
    
    const iconMap = {
        // Documents
        'pdf': 'fas fa-file-pdf',
        'doc': 'fas fa-file-word',
        'docx': 'fas fa-file-word',
        'txt': 'fas fa-file-alt',
        'rtf': 'fas fa-file-alt',
        'odt': 'fas fa-file-alt',
        
        // Spreadsheets
        'xls': 'fas fa-file-excel',
        'xlsx': 'fas fa-file-excel',
        'csv': 'fas fa-file-csv',
        'ods': 'fas fa-file-excel',
        
        // Presentations
        'ppt': 'fas fa-file-powerpoint',
        'pptx': 'fas fa-file-powerpoint',
        'odp': 'fas fa-file-powerpoint',
        
        // Images
        'jpg': 'fas fa-file-image',
        'jpeg': 'fas fa-file-image',
        'png': 'fas fa-file-image',
        'gif': 'fas fa-file-image',
        'bmp': 'fas fa-file-image',
        'svg': 'fas fa-file-image',
        'webp': 'fas fa-file-image',
        
        // Audio
        'mp3': 'fas fa-file-audio',
        'wav': 'fas fa-file-audio',
        'flac': 'fas fa-file-audio',
        'aac': 'fas fa-file-audio',
        'ogg': 'fas fa-file-audio',
        'm4a': 'fas fa-file-audio',
        
        // Video
        'mp4': 'fas fa-file-video',
        'avi': 'fas fa-file-video',
        'mkv': 'fas fa-file-video',
        'mov': 'fas fa-file-video',
        'wmv': 'fas fa-file-video',
        'flv': 'fas fa-file-video',
        'webm': 'fas fa-file-video',
        
        // Code
        'js': 'fas fa-file-code',
        'html': 'fas fa-file-code',
        'css': 'fas fa-file-code',
        'py': 'fas fa-file-code',
        'java': 'fas fa-file-code',
        'cpp': 'fas fa-file-code',
        'c': 'fas fa-file-code',
        'php': 'fas fa-file-code',
        'rb': 'fas fa-file-code',
        'go': 'fas fa-file-code',
        'rs': 'fas fa-file-code',
        'swift': 'fas fa-file-code',
        'kt': 'fas fa-file-code',
        'ts': 'fas fa-file-code',
        'jsx': 'fas fa-file-code',
        'vue': 'fas fa-file-code',
        'sql': 'fas fa-file-code',
        'json': 'fas fa-file-code',
        'xml': 'fas fa-file-code',
        'yaml': 'fas fa-file-code',
        'yml': 'fas fa-file-code',
        
        // Archives
        'zip': 'fas fa-file-archive',
        'rar': 'fas fa-file-archive',
        '7z': 'fas fa-file-archive',
        'tar': 'fas fa-file-archive',
        'gz': 'fas fa-file-archive',
        'bz2': 'fas fa-file-archive'
    };
    
    return iconMap[extension] || 'fas fa-file';
}

// Function to format file size
function formatFileSize(bytes) {
    if (bytes === 0) return '0 Bytes';
    
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
}

// Function to show error message
function showError(message) {
    const responseElement = document.getElementById("json-response");
    responseElement.innerHTML = `<span class="error">Error: ${message}</span>`;
    console.error("API Error:", message);
}

// Function to show loading indicator
function showLoading() {
    document.getElementById("loading-indicator").style.display = "flex";
}

// Function to hide loading indicator
function hideLoading() {
    document.getElementById("loading-indicator").style.display = "none";
}