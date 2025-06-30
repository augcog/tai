// Constants for API URLs
const BASE_URL = "/api/files";

// Store current file list for reference
let currentFiles = [];

// Store API token
let apiToken = null;

// After DOM content is loaded, initialize the application
document.addEventListener("DOMContentLoaded", () => {
    // Add event listeners to forms
    document.getElementById("files-form").addEventListener("submit", handleFilesSubmit);
    document.getElementById("file-metadata-form").addEventListener("submit", handleFileMetadataSubmit);
    document.getElementById("file-download-form").addEventListener("submit", handleFileDownloadSubmit);
    document.getElementById("stats-form").addEventListener("submit", handleStatsSubmit);
    document.getElementById("clear-response-btn").addEventListener("click", clearResponse);
    document.getElementById("set-api-token-btn").addEventListener("click", handleSetApiToken);
    
    // Initialize by listing all files
    listFiles();
});

// Handler for listing files
async function handleFilesSubmit(event) {
    event.preventDefault();
    
    const courseCode = document.getElementById("course-code-input").value.trim();
    const category = document.getElementById("category-input").value;
    const search = document.getElementById("search-input").value.trim();
    const page = parseInt(document.getElementById("page-input").value) || 1;
    const limit = parseInt(document.getElementById("limit-input").value) || 100;
    
    await listFiles(courseCode, category, search, page, limit);
}

// Handler for file metadata
async function handleFileMetadataSubmit(event) {
    event.preventDefault();
    const fileId = document.getElementById("file-id-input").value.trim();
    
    if (!fileId) {
        showError("Please enter a file UUID");
        return;
    }
    
    await getFileMetadata(fileId);
}

// Handler for file download
async function handleFileDownloadSubmit(event) {
    event.preventDefault();
    const fileId = document.getElementById("download-file-id-input").value.trim();
    
    if (!fileId) {
        showError("Please enter a file UUID");
        return;
    }
    
    await downloadFile(fileId);
}

// Handler for file statistics
async function handleStatsSubmit(event) {
    event.preventDefault();
    await getFileStats();
}

// Handler for setting API token
function handleSetApiToken() {
    const tokenInput = document.getElementById("api-token-input");
    const token = tokenInput.value.trim();
    const statusDiv = document.getElementById("token-status");
    
    if (!token) {
        statusDiv.innerHTML = '<span style="color: red;"><i class="fas fa-exclamation-triangle"></i> Please enter a token</span>';
        return;
    }
    
    // Store the token
    apiToken = token;
    
    // Clear the input for security
    tokenInput.value = "";
    
    // Show success message
    statusDiv.innerHTML = '<span style="color: green;"><i class="fas fa-check-circle"></i> API token set successfully</span>';
    
    console.log("API token set successfully");
}

// Helper function to get headers with API token
function getHeaders() {
    const headers = {
        "Accept": "application/json"
    };
    
    if (apiToken) {
        headers["Authorization"] = `Bearer ${apiToken}`;
    }
    
    return headers;
}

// Function to list files with filtering
async function listFiles(courseCode = "", category = "", search = "", page = 1, limit = 100) {
    showLoading();
    
    try {
        // Build query parameters
        const params = new URLSearchParams();
        if (courseCode) params.append("course_code", courseCode);
        if (category) params.append("category", category);
        if (search) params.append("search", search);
        params.append("page", page.toString());
        params.append("limit", limit.toString());
        
        const queryString = params.toString();
        const url = queryString ? `${BASE_URL}?${queryString}` : BASE_URL;
        
        console.log("Fetching files from:", url);
        
        const response = await fetch(url, {
            method: "GET",
            headers: getHeaders()
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to list files");
        }
        
        console.log("Files API Response:", data);
        
        // Update UI with the file list
        renderFileList(data);
        displayJsonResponse(data);
        
    } catch (error) {
        console.error("Error listing files:", error);
        showError(`Error listing files: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get file metadata by UUID
async function getFileMetadata(fileId) {
    showLoading();
    
    try {
        console.log("Fetching metadata for file:", fileId);
        
        const response = await fetch(`${BASE_URL}/${fileId}`, {
            method: "GET",
            headers: getHeaders()
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to get file metadata");
        }
        
        console.log("File metadata:", data);
        
        // Display metadata in JSON response
        displayJsonResponse(data);
        
        // Show file preview with metadata
        renderFilePreview(data);
        
    } catch (error) {
        console.error("Error getting file metadata:", error);
        showError(`Error getting file metadata: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to download file by UUID
async function downloadFile(fileId) {
    showLoading();
    
    try {
        console.log("Downloading file:", fileId);
        
        const response = await fetch(`${BASE_URL}/${fileId}/download`, {
            method: "GET",
            headers: getHeaders()
        });
        
        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.detail || "Failed to download file");
        }
        
        // Get filename from response headers or use default
        const contentDisposition = response.headers.get('Content-Disposition');
        let filename = 'download';
        if (contentDisposition) {
            const filenameMatch = contentDisposition.match(/filename="?([^"]+)"?/);
            if (filenameMatch) {
                filename = filenameMatch[1];
            }
        }
        
        // Download the file
        const blob = await response.blob();
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url);
        document.body.removeChild(a);
        
        displayJsonResponse({
            message: "File downloaded successfully",
            filename: filename,
            size: blob.size
        });
        
    } catch (error) {
        console.error("Error downloading file:", error);
        showError(`Error downloading file: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get file statistics
async function getFileStats() {
    showLoading();
    
    try {
        console.log("Fetching file statistics");
        
        const response = await fetch(`${BASE_URL}/stats/summary`, {
            method: "GET",
            headers: getHeaders()
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to get file statistics");
        }
        
        console.log("File statistics:", data);
        
        // Display stats in JSON response
        displayJsonResponse(data);
        
        // Show stats in a formatted way
        renderStatsDisplay(data);
        
    } catch (error) {
        console.error("Error getting file statistics:", error);
        showError(`Error getting file statistics: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to render file list in UI
function renderFileList(data) {
    const fileListElement = document.getElementById("file-list");
    fileListElement.innerHTML = "";
    
    if (!data.files || data.files.length === 0) {
        fileListElement.innerHTML = `
            <div class="empty-state">
                <i class="fas fa-folder-open"></i>
                <p>No files found</p>
            </div>
        `;
        return;
    }
    
    // Store current files for reference
    currentFiles = data.files;
    
    // Create list header with pagination info
    const listHeader = document.createElement('div');
    listHeader.className = 'list-header';
    listHeader.innerHTML = `
        <span class="file-count">${data.files.length} files (Page ${data.page} of ${Math.ceil(data.total_count / data.limit)})</span>
        <span class="total-count">Total: ${data.total_count}</span>
    `;
    fileListElement.appendChild(listHeader);
    
    // Show applied filters if any
    if (data.filters_applied && Object.values(data.filters_applied).some(v => v)) {
        const filtersDiv = document.createElement('div');
        filtersDiv.className = 'applied-filters';
        const activeFilters = Object.entries(data.filters_applied)
            .filter(([key, value]) => value)
            .map(([key, value]) => `${key}: ${value}`)
            .join(', ');
        filtersDiv.innerHTML = `<small>Filters: ${activeFilters}</small>`;
        fileListElement.appendChild(filtersDiv);
    }
    
    // Create file items
    const listContainer = document.createElement('div');
    listContainer.className = 'file-items';
    
    data.files.forEach(file => {
        const fileItem = document.createElement('div');
        fileItem.className = 'file-item file';
        
        const iconClass = getFileIcon(file.filename);
        
        fileItem.innerHTML = `
            <i class="fas ${iconClass}"></i>
            <div class="file-details">
                <span class="file-name" title="${file.filename}">${file.title || file.filename}</span>
                <span class="file-info">
                    ${formatFileSize(file.size_bytes)}
                    ${file.course ? `• ${file.course}` : ''}
                    ${file.category ? `• ${file.category}` : ''}
                </span>
                <span class="file-uuid" title="UUID: ${file.uuid}">UUID: ${file.uuid.substring(0, 8)}...</span>
            </div>
        `;
        
        // Add click event to populate UUID fields and get metadata
        fileItem.addEventListener('click', () => {
            // Populate UUID input fields
            document.getElementById("file-id-input").value = file.uuid;
            document.getElementById("download-file-id-input").value = file.uuid;
            
            // Show file metadata
            getFileMetadata(file.uuid);
        });
        
        listContainer.appendChild(fileItem);
    });
    
    fileListElement.appendChild(listContainer);
    
    // Add pagination controls if needed
    if (data.has_prev || data.has_next) {
        const paginationDiv = document.createElement('div');
        paginationDiv.className = 'pagination-controls';
        
        let paginationHTML = '';
        if (data.has_prev) {
            paginationHTML += `<button class="btn-small" onclick="changePage(${data.page - 1})">Previous</button>`;
        }
        paginationHTML += `<span>Page ${data.page}</span>`;
        if (data.has_next) {
            paginationHTML += `<button class="btn-small" onclick="changePage(${data.page + 1})">Next</button>`;
        }
        
        paginationDiv.innerHTML = paginationHTML;
        fileListElement.appendChild(paginationDiv);
    }
}

// Function to change page
function changePage(newPage) {
    document.getElementById("page-input").value = newPage;
    document.getElementById("files-form").dispatchEvent(new Event('submit'));
}

// Function to render file preview
function renderFilePreview(fileData) {
    const previewElement = document.getElementById("preview-content");
    
    let previewHTML = `
        <div class="file-preview">
            <div class="file-header">
                <h3>${fileData.title || fileData.filename}</h3>
                <p class="file-meta">
                    <strong>UUID:</strong> ${fileData.uuid}<br>
                    <strong>Size:</strong> ${formatFileSize(fileData.size_bytes)}<br>
                    <strong>Type:</strong> ${fileData.mime_type}<br>
                    ${fileData.course ? `<strong>Course:</strong> ${fileData.course}<br>` : ''}
                    ${fileData.category ? `<strong>Category:</strong> ${fileData.category}<br>` : ''}
                    <strong>Created:</strong> ${new Date(fileData.created_at).toLocaleString()}
                </p>
            </div>
    `;
    
    // Show preview based on file type
    if (fileData.mime_type.startsWith('image/')) {
        previewHTML += `
            <div class="image-preview">
                <p><em>Image preview not available in metadata view. Use download to view the image.</em></p>
            </div>
        `;
    } else if (fileData.mime_type.startsWith('text/')) {
        previewHTML += `
            <div class="text-preview">
                <p><em>Text preview not available in metadata view. Use download to view the content.</em></p>
            </div>
        `;
    } else {
        previewHTML += `
            <div class="file-preview-info">
                <p><em>Preview not available for this file type. Use download to access the file.</em></p>
            </div>
        `;
    }
    
    previewHTML += '</div>';
    previewElement.innerHTML = previewHTML;
}

// Function to render statistics display
function renderStatsDisplay(stats) {
    const previewElement = document.getElementById("preview-content");
    
    let statsHTML = `
        <div class="stats-display">
            <h3>File System Statistics</h3>
            <div class="stats-grid">
                <div class="stat-item">
                    <strong>Total Files:</strong> ${stats.total_files}
                </div>
                <div class="stat-item">
                    <strong>Base Directory:</strong> ${stats.base_directory}
                </div>
                <div class="stat-item">
                    <strong>Auto Discovery:</strong> ${stats.auto_discovery}
                </div>
                <div class="stat-item">
                    <strong>Last Updated:</strong> ${new Date(stats.last_updated).toLocaleString()}
                </div>
            </div>
    `;
    
    if (stats.courses && Object.keys(stats.courses).length > 0) {
        statsHTML += `
            <div class="courses-breakdown">
                <h4>Files by Course:</h4>
                <ul>
        `;
        
        Object.entries(stats.courses).forEach(([course, count]) => {
            statsHTML += `<li><strong>${course}:</strong> ${count} files</li>`;
        });
        
        statsHTML += `
                </ul>
            </div>
        `;
    }
    
    statsHTML += '</div>';
    previewElement.innerHTML = statsHTML;
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
        'pdf': 'fa-file-pdf',
        'doc': 'fa-file-word',
        'docx': 'fa-file-word',
        'txt': 'fa-file-alt',
        'rtf': 'fa-file-alt',
        
        // Spreadsheets
        'xls': 'fa-file-excel',
        'xlsx': 'fa-file-excel',
        'csv': 'fa-file-csv',
        
        // Presentations
        'ppt': 'fa-file-powerpoint',
        'pptx': 'fa-file-powerpoint',
        
        // Images
        'jpg': 'fa-file-image',
        'jpeg': 'fa-file-image',
        'png': 'fa-file-image',
        'gif': 'fa-file-image',
        'bmp': 'fa-file-image',
        'svg': 'fa-file-image',
        
        // Videos
        'mp4': 'fa-file-video',
        'avi': 'fa-file-video',
        'mov': 'fa-file-video',
        'wmv': 'fa-file-video',
        'flv': 'fa-file-video',
        'webm': 'fa-file-video',
        
        // Audio
        'mp3': 'fa-file-audio',
        'wav': 'fa-file-audio',
        'flac': 'fa-file-audio',
        'aac': 'fa-file-audio',
        'ogg': 'fa-file-audio',
        
        // Archives
        'zip': 'fa-file-archive',
        'rar': 'fa-file-archive',
        '7z': 'fa-file-archive',
        'tar': 'fa-file-archive',
        'gz': 'fa-file-archive',
        
        // Code
        'js': 'fa-file-code',
        'html': 'fa-file-code',
        'css': 'fa-file-code',
        'py': 'fa-file-code',
        'java': 'fa-file-code',
        'cpp': 'fa-file-code',
        'c': 'fa-file-code',
        'php': 'fa-file-code',
        'json': 'fa-file-code',
        'xml': 'fa-file-code'
    };
    
    return iconMap[extension] || 'fa-file';
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
    const previewElement = document.getElementById("preview-content");
    previewElement.innerHTML = `
        <div class="error-message">
            <i class="fas fa-exclamation-triangle"></i>
            <p>${message}</p>
        </div>
    `;
}

// Function to show loading indicator
function showLoading() {
    document.getElementById("loading-indicator").style.display = "flex";
}

// Function to hide loading indicator
function hideLoading() {
    document.getElementById("loading-indicator").style.display = "none";
}