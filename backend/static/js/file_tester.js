// Constants for API URLs and common values
const BASE_URL = "/v1/local-files";
const AUTH_TOKEN = "Bearer your_access_token_here";

// Cache for folder hierarchy to avoid redundant API calls
let hierarchyCache = null;
let currentFiles = []; // Store current file list for reference

// After DOM content is loaded, initialize the application
document.addEventListener("DOMContentLoaded", () => {
    // Add event listeners to various forms and inputs
    document.getElementById("directory-form").addEventListener("submit", handleDirectorySubmit);
    document.getElementById("file-form").addEventListener("submit", handleFileSubmit);
    document.getElementById("categories-form").addEventListener("submit", handleCategoriesSubmit);
    document.getElementById("folders-form").addEventListener("submit", handleFoldersSubmit);
    document.getElementById("hierarchy-form").addEventListener("submit", handleHierarchySubmit);
    document.getElementById("clear-response-btn").addEventListener("click", clearResponse);
    
    // Initialize by listing root directory files
    listFiles("");
});

// Handler for listing files in a directory
async function handleDirectorySubmit(event) {
    event.preventDefault();
    const directoryPath = document.getElementById("directory-input").value.trim();
    await listFiles(directoryPath);
}

// Handler for retrieving a specific file
async function handleFileSubmit(event) {
    event.preventDefault();
    const filePath = document.getElementById("file-input").value.trim();
    
    if (!filePath) {
        showError("Please enter a file path");
        return;
    }
    
    await getFile(filePath);
}

// Handler for retrieving categories
async function handleCategoriesSubmit(event) {
    event.preventDefault();
    await getCategories();
}

// Handler for retrieving folders
async function handleFoldersSubmit(event) {
    event.preventDefault();
    await getFolders();
}

// Handler for retrieving folder hierarchy
async function handleHierarchySubmit(event) {
    event.preventDefault();
    const directory = document.getElementById("hierarchy-directory-input").value.trim();
    const maxDepth = document.getElementById("max-depth-input").value.trim();
    await getHierarchy(directory, maxDepth);
}

// Function to list files in a directory
async function listFiles(directoryPath = "") {
    showLoading();
    
    try {
        const queryParams = directoryPath ? `?directory=${encodeURIComponent(directoryPath)}` : "";
        const response = await fetch(`${BASE_URL}${queryParams}`, {
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
        
        // Update UI with the file list
        renderFileList(data, directoryPath);
        displayJsonResponse(data);
        
    } catch (error) {
        showError(`Error listing files: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get file content by path
async function getFile(filePath) {
    showLoading();
    
    try {
        // Handle potential empty or invalid paths
        if (!filePath || filePath.trim() === '') {
            throw new Error("No file path provided");
        }
        
        console.log("Fetching file:", filePath);
        
        // Properly encode the file path - remove any leading slashes to match backend expectations
        let cleanPath = filePath;
        while (cleanPath.startsWith('/')) {
            cleanPath = cleanPath.substring(1);
        }
        
        const encodedPath = encodeURIComponent(cleanPath);
        const url = `${BASE_URL}/${encodedPath}`;
        
        console.log("Request URL:", url);
        
        const response = await fetch(url, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN
                // No Accept header to allow any content type
            }
        });
        
        console.log("Response status:", response.status);
        
        if (!response.ok) {
            let errorMessage = "Failed to retrieve file";
            
            try {
                // Try to get error details as JSON
                const errorData = await response.json();
                errorMessage = errorData.detail || errorMessage;
            } catch (e) {
                // If not JSON, try to get as text
                try {
                    const errorText = await response.text();
                    if (errorText) errorMessage = errorText;
                } catch (textError) {
                    // If that fails too, use status text
                    errorMessage = `Error ${response.status}: ${response.statusText}`;
                }
            }
            
            throw new Error(errorMessage);
        }
        
        // Get content type and other headers
        const contentType = response.headers.get("Content-Type") || "";
        console.log("File response content type:", contentType);
        
        // Create a file data object to render
        let fileData = {
            name: filePath.split('/').pop(),
            path: filePath,
            type: contentType,
            size: parseInt(response.headers.get("Content-Length") || "0")
        };
        
        if (contentType.includes("application/json")) {
            // Handle JSON response
            const jsonData = await response.json();
            console.log("JSON file data:", jsonData);
            displayJsonResponse(jsonData);
            
            // If the JSON contains file content, use that for preview
            if (jsonData.content) {
                fileData = { ...fileData, ...jsonData };
            } else if (jsonData.data && jsonData.data.content) {
                fileData = { ...fileData, ...jsonData.data };
            }
        } else if (contentType.includes("text/")) {
            // Handle text response
            const textContent = await response.text();
            console.log("Text file length:", textContent.length);
            displayJsonResponse({ 
                message: "Text file loaded successfully", 
                size: textContent.length,
                type: contentType
            });
            
            // Store text content for preview
            fileData.content = btoa(unescape(encodeURIComponent(textContent))); // Handle UTF-8 characters properly
        } else {
            // Handle binary response (images, PDFs, etc.)
            const blob = await response.blob();
            console.log("Binary file size:", blob.size);
            
            // Convert blob to base64 for preview
            try {
                fileData.content = await blobToBase64(blob);
                fileData.size = blob.size;
                
                displayJsonResponse({ 
                    message: "Binary file loaded successfully", 
                    size: blob.size,
                    type: contentType
                });
            } catch (error) {
                console.error("Error converting blob to base64:", error);
                throw new Error("Failed to process binary file: " + error.message);
            }
        }
        
        console.log("File data prepared for preview:", fileData);
        
        // Update UI with file data
        renderFilePreview(fileData);
        
    } catch (error) {
        console.error("File retrieval error:", error);
        showError(`Error retrieving file: ${error.message}`);
        
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

// Helper function to convert Blob to base64
function blobToBase64(blob) {
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => {
            const base64Result = reader.result;
            // Get just the base64 part without the data URL prefix
            if (base64Result.includes(',')) {
                resolve(base64Result.split(',')[1]);
            } else {
                resolve(base64Result);
            }
        };
        reader.onerror = (error) => reject(error);
        reader.readAsDataURL(blob);
    });
}

// Function to get categories
async function getCategories() {
    showLoading();
    
    try {
        const response = await fetch(`${BASE_URL}/categories`, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN,
                "Accept": "application/json"
            }
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to retrieve categories");
        }
        
        // Update UI with categories data
        displayJsonResponse(data);
        
        // Optional: Display categories in a structured way
        renderCategories(data.categories);
        
    } catch (error) {
        showError(`Error retrieving categories: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get folders
async function getFolders() {
    showLoading();
    
    try {
        const response = await fetch(`${BASE_URL}/folders`, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN,
                "Accept": "application/json"
            }
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to retrieve folders");
        }
        
        // Update UI with folders data
        displayJsonResponse(data);
        
        // Optional: Display folders in a structured way
        renderFolders(data.folders);
        
    } catch (error) {
        showError(`Error retrieving folders: ${error.message}`);
    } finally {
        hideLoading();
    }
}

// Function to get folder hierarchy
async function getHierarchy(directory = '', maxDepth = -1) {
    showLoading();
    
    try {
        const url = `${BASE_URL}/hierarchy?directory=${encodeURIComponent(directory)}&max_depth=${maxDepth}`;
        const response = await fetch(url, {
            method: "GET",
            headers: {
                "Authorization": AUTH_TOKEN,
                "Accept": "application/json"
            }
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.detail || "Failed to retrieve hierarchy");
        }
        
        // Cache the result for future use
        hierarchyCache = data;
        displayJsonResponse(data);

        return data;
        
    } catch (error) {
        console.error(`Error retrieving hierarchy: ${error.message}`);
        showError(`Error retrieving hierarchy: ${error.message}`);
        return null;
    } finally {
        hideLoading();
    }
}

// Function to render file list in UI
function renderFileList(data, directory) {
    const fileListElement = document.getElementById("file-list");
    fileListElement.innerHTML = "";
    
    // Extract files array from data based on response structure
    let files = [];
    
    if (data.files && Array.isArray(data.files)) {
        files = data.files;
    } else if (data.data && Array.isArray(data.data)) {
        files = data.data;
    } else if (Array.isArray(data)) {
        files = data;
    }
    
    // Store current files for reference
    currentFiles = files;
    
    if (!files || files.length === 0) {
        fileListElement.innerHTML = `
            <div class="empty-state">
                <i class="fas fa-folder-open"></i>
                <p>No files found in ${directory || 'root directory'}</p>
            </div>
        `;
        return;
    }
    
    // Create list header
    const listHeader = document.createElement('div');
    listHeader.className = 'list-header';
    listHeader.innerHTML = `
        <span class="directory-path">Directory: ${directory || 'root'}</span>
        <span class="file-count">${files.length} files</span>
    `;
    fileListElement.appendChild(listHeader);
    
    // Create file items
    const listContainer = document.createElement('div');
    listContainer.className = 'file-items';
    
    files.forEach(file => {
        // Log each file object to see its structure
        console.log("File item:", file);
        
        // Get file properties, adapting to potential different API formats
        const fileName = file.name || file.filename || file.file_name || "Unknown";
        const isDirectory = file.is_dir || file.is_directory || file.isDirectory || false;
        const fileSize = file.size || file.size_bytes || file.file_size || file.filesize || 0;
        
        // Get file path, preferring a provided path, falling back to constructing one
        let filePath = file.path || file.file_path || file.filepath;
        if (!filePath) {
            // Construct path from directory and filename
            filePath = directory ? `${directory}/${fileName}` : fileName;
        }
        
        const fileItem = document.createElement('div');
        fileItem.className = `file-item ${isDirectory ? 'directory' : 'file'}`;
        
        // Safely get the icon
        let iconClass = isDirectory ? 'fa-folder' : 'fa-file';
        if (!isDirectory && fileName) {
            iconClass = getFileIcon(fileName);
        }
        
        fileItem.innerHTML = `
            <i class="fas ${iconClass}"></i>
            <div class="file-details">
                <span class="file-name">${fileName}</span>
                <span class="file-info">${isDirectory ? 'Directory' : formatFileSize(fileSize)}</span>
            </div>
        `;
        
        // Add click event for files and directories
        fileItem.addEventListener('click', () => {
            if (isDirectory) {
                // Navigate to directory
                const newPath = directory ? `${directory}/${fileName}` : fileName;
                document.getElementById("directory-input").value = newPath;
                listFiles(newPath);
            } else {
                try {
                    console.log("Selected file for preview:", filePath);
                    
                    // Set the path in the input field
                    document.getElementById("file-input").value = filePath;
                    
                    // Show immediate feedback in the preview panel
                    const previewElement = document.getElementById("preview-content");
                    previewElement.innerHTML = `
                        <div class="loading-state">
                            <i class="fas fa-spinner fa-spin"></i>
                            <p>Loading file preview...</p>
                        </div>
                    `;
                    
                    // Fetch and preview the file with the full path
                    getFile(filePath);
                } catch (error) {
                    console.error("Error in file click handler:", error);
                    showError(`Error handling file: ${error.message}`);
                }
            }
        });
        
        listContainer.appendChild(fileItem);
    });
    
    fileListElement.appendChild(listContainer);
}

// Function to render categories
function renderCategories(categories) {
    const fileListElement = document.getElementById("file-list");
    fileListElement.innerHTML = "";
    
    if (!categories || categories.length === 0) {
        fileListElement.innerHTML = `
            <div class="empty-state">
                <i class="fas fa-tags"></i>
                <p>No categories found</p>
            </div>
        `;
        return;
    }
    
    // Create list header
    const listHeader = document.createElement('div');
    listHeader.className = 'list-header';
    listHeader.innerHTML = `
        <span class="category-title">Categories</span>
        <span class="category-count">${categories.length} categories</span>
    `;
    fileListElement.appendChild(listHeader);
    
    // Create category items
    const listContainer = document.createElement('div');
    listContainer.className = 'file-items';
    
    categories.forEach(category => {
        const categoryItem = document.createElement('div');
        categoryItem.className = 'file-item category';
        
        categoryItem.innerHTML = `
            <i class="fas fa-tag"></i>
            <div class="file-details">
                <span class="file-name">${category.name || 'Unknown'}</span>
                <span class="file-info">${category.file_count || 0} files</span>
            </div>
        `;
        
        listContainer.appendChild(categoryItem);
    });
    
    fileListElement.appendChild(listContainer);
}

// Function to render folders
function renderFolders(folders) {
    const fileListElement = document.getElementById("file-list");
    fileListElement.innerHTML = "";
    
    if (!folders || folders.length === 0) {
        fileListElement.innerHTML = `
            <div class="empty-state">
                <i class="fas fa-folder"></i>
                <p>No folders found</p>
            </div>
        `;
        return;
    }
    
    // Create list header
    const listHeader = document.createElement('div');
    listHeader.className = 'list-header';
    listHeader.innerHTML = `
        <span class="folder-title">Folders</span>
        <span class="folder-count">${folders.length} folders</span>
    `;
    fileListElement.appendChild(listHeader);
    
    // Create folder items
    const listContainer = document.createElement('div');
    listContainer.className = 'file-items';
    
    folders.forEach(folder => {
        const folderItem = document.createElement('div');
        folderItem.className = 'file-item folder';
        
        folderItem.innerHTML = `
            <i class="fas fa-folder"></i>
            <div class="file-details">
                <span class="file-name">${folder.name || 'Unknown'}</span>
                <span class="file-info">${folder.file_count || 0} files</span>
            </div>
        `;
        
        // Add click event for folders
        folderItem.addEventListener('click', () => {
            document.getElementById("directory-input").value = folder.path;
            listFiles(folder.path);
        });
        
        listContainer.appendChild(folderItem);
    });
    
    fileListElement.appendChild(listContainer);
}

// Function to render file preview
function renderFilePreview(fileData) {
    const previewElement = document.getElementById("preview-content");
    
    // Reset preview area
    previewElement.innerHTML = "";
    
    if (!fileData) {
        previewElement.innerHTML = `
            <div class="empty-state">
                <i class="fas fa-file"></i>
                <p>No preview available</p>
            </div>
        `;
        return;
    }
    
    console.log("Rendering preview for:", fileData);
    
    // File information header
    const fileInfoElement = document.createElement("div");
    fileInfoElement.className = "file-info-header";
    fileInfoElement.innerHTML = `
        <div><strong>Name:</strong> ${fileData.name || "Unknown"}</div>
        <div><strong>Type:</strong> ${fileData.type || "Unknown"}</div>
        <div><strong>Size:</strong> ${formatFileSize(fileData.size || 0)}</div>
    `;
    
    previewElement.appendChild(fileInfoElement);
    
    // Preview content based on file type
    const contentElement = document.createElement("div");
    contentElement.className = "preview-content-area";
    
    // Handle missing content
    if (!fileData.content) {
        contentElement.innerHTML = `
            <div class="empty-state">
                <i class="fas fa-file-alt"></i>
                <p>Content not available for preview</p>
                <p class="details">The file appears to be empty or its content cannot be displayed.</p>
            </div>
        `;
    } 
    // Handle image files
    else if (fileData.type && fileData.type.includes("image/")) {
        try {
            contentElement.innerHTML = `
                <div class="image-preview">
                    <img src="data:${fileData.type};base64,${fileData.content}" alt="${fileData.name || 'Image'}" />
                </div>
            `;
        } catch (e) {
            contentElement.innerHTML = `
                <div class="error-message">
                    <i class="fas fa-exclamation-triangle"></i>
                    <p>Error loading image: ${e.message}</p>
                </div>
            `;
        }
    } 
    // Handle PDF files
    else if (fileData.type && fileData.type.includes("application/pdf")) {
        const pdfContainer = document.createElement("div");
        pdfContainer.className = "pdf-preview";
        pdfContainer.innerHTML = `
            <i class="fas fa-file-pdf"></i>
            <p>PDF preview available</p>
            <a href="data:application/pdf;base64,${fileData.content}" download="${fileData.name || 'document.pdf'}" class="btn btn-primary">Download PDF</a>
        `;
        contentElement.appendChild(pdfContainer);
    } 
    // Handle text and other files
    else {
        try {
            // Attempt to decode base64 content
            let decodedContent = '';
            
            try {
                decodedContent = atob(fileData.content);
            } catch (e) {
                console.warn("Failed to decode as base64, using content directly:", e);
                // If not base64, try using the content directly
                decodedContent = fileData.content;
            }
            
            // Create formatted text area
            const codeElement = document.createElement("pre");
            codeElement.className = "code-preview";
            codeElement.textContent = decodedContent;
            contentElement.appendChild(codeElement);
        } catch (error) {
            console.error("Preview rendering error:", error);
            contentElement.innerHTML = `
                <div class="error-message">
                    <i class="fas fa-exclamation-triangle"></i>
                    <p>Unable to preview this file type: ${error.message}</p>
                </div>
            `;
        }
    }
    
    previewElement.appendChild(contentElement);
}

// Function to display JSON response
function displayJsonResponse(data) {
    const responseElement = document.getElementById("json-response");
    // Pretty print the JSON with syntax highlighting
    responseElement.innerHTML = syntaxHighlight(JSON.stringify(data, null, 2));
}

// Function to clear the JSON response
function clearResponse() {
    const responseElement = document.getElementById("json-response");
    responseElement.innerHTML = '';
}

// Function for JSON syntax highlighting
function syntaxHighlight(json) {
    json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    return json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, match => {
        let cls = 'json-number';
        if (/^"/.test(match)) {
            if (/:$/.test(match)) {
                cls = 'json-key';
            } else {
                cls = 'json-string';
            }
        } else if (/true|false/.test(match)) {
            cls = 'json-boolean';
        } else if (/null/.test(match)) {
            cls = 'json-null';
        }
        return `<span class="${cls}">${match}</span>`;
    });
}

// Function to determine file icon based on file extension
function getFileIcon(fileName) {
    if (!fileName) return 'fa-file';
    
    try {
        const extension = fileName.split('.').pop().toLowerCase();
        
        const iconMap = {
            // Documents
            'pdf': 'fa-file-pdf',
            'doc': 'fa-file-word', 'docx': 'fa-file-word',
            'xls': 'fa-file-excel', 'xlsx': 'fa-file-excel',
            'ppt': 'fa-file-powerpoint', 'pptx': 'fa-file-powerpoint',
            
            // Images
            'jpg': 'fa-file-image', 'jpeg': 'fa-file-image',
            'png': 'fa-file-image', 'gif': 'fa-file-image',
            'svg': 'fa-file-image', 'bmp': 'fa-file-image',
            
            // Code
            'html': 'fa-file-code', 'css': 'fa-file-code',
            'js': 'fa-file-code', 'json': 'fa-file-code',
            'py': 'fa-file-code', 'java': 'fa-file-code',
            'c': 'fa-file-code', 'cpp': 'fa-file-code',
            'php': 'fa-file-code',
            
            // Archives
            'zip': 'fa-file-archive', 'rar': 'fa-file-archive',
            '7z': 'fa-file-archive', 'tar': 'fa-file-archive',
            'gz': 'fa-file-archive',
            
            // Text
            'txt': 'fa-file-alt', 'md': 'fa-file-alt',
            'rtf': 'fa-file-alt',
            
            // Audio/Video
            'mp3': 'fa-file-audio', 'wav': 'fa-file-audio',
            'ogg': 'fa-file-audio',
            'mp4': 'fa-file-video', 'avi': 'fa-file-video',
            'mov': 'fa-file-video', 'wmv': 'fa-file-video'
        };
        
        return iconMap[extension] || 'fa-file';
    } catch (error) {
        // If any error occurs while getting the icon, return default file icon
        console.error("Error determining file icon:", error);
        return 'fa-file';
    }
}

// Function to format file size into human-readable format
function formatFileSize(bytes) {
    if (!bytes && bytes !== 0) return 'Unknown';
    if (bytes === 0) return '0 Bytes';
    
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
}

// Show error message
function showError(message) {
    const responseElement = document.getElementById("json-response");
    responseElement.innerHTML = `<div class="error-message">${message}</div>`;
}

// Show loading indicator
function showLoading() {
    document.getElementById("loading-indicator").style.display = "flex";
}

// Hide loading indicator
function hideLoading() {
    document.getElementById("loading-indicator").style.display = "none";
}