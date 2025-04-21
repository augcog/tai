// Constants
const API_BASE_URL = '/v1/local-files';
const MOCK_AUTH_TOKEN = 'Bearer mock_test_token';

// DOM Elements
document.addEventListener('DOMContentLoaded', () => {
    // Form elements
    const listFilesForm = document.getElementById('listFilesForm');
    const directoryInput = document.getElementById('directoryInput');
    const getFileForm = document.getElementById('getFileForm');
    const filePathInput = document.getElementById('filePathInput');
    
    // Display elements
    const fileList = document.getElementById('fileList');
    const jsonResponse = document.getElementById('jsonResponse');
    const previewContent = document.getElementById('previewContent');
    const loadingIndicator = document.getElementById('loadingIndicator');
    
    // Event listeners
    listFilesForm.addEventListener('submit', handleListFiles);
    getFileForm.addEventListener('submit', handleGetFile);
    
    // Initial state - show welcome message
    showApiResponse({ 
        message: 'Welcome to the Local File Retrieval API Tester',
        note: 'API authentication is handled automatically in development mode'
    });
    
    /**
     * Handle listing files from the API
     * @param {Event} event - Form submit event
     */
    async function handleListFiles(event) {
        event.preventDefault();
        
        const directory = directoryInput.value.trim();
        
        // Show loading state
        toggleLoading(true);
        resetFileList();
        
        try {
            // Build query parameters
            const params = new URLSearchParams();
            if (directory) {
                params.append('directory', directory);
            }
            
            // Fetch files from API with auth header
            const response = await fetchWithAuth(`${API_BASE_URL}?${params.toString()}`);
            
            if (!response.ok) {
                const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
                throw new Error(errorData.detail || `HTTP error ${response.status}`);
            }
            
            const data = await response.json();
            
            // Display API response
            showApiResponse(data);
            
            // Display files in the list
            if (data.files && data.files.length > 0) {
                renderFileList(data.files);
            } else {
                showEmptyFileList();
            }
        } catch (error) {
            console.error('Error fetching files:', error);
            showApiResponse({ error: error.message });
            showEmptyFileList('Error fetching files');
        } finally {
            toggleLoading(false);
        }
    }
    
    /**
     * Handle getting a specific file from the API
     * @param {Event} event - Form submit event
     */
    async function handleGetFile(event) {
        event.preventDefault();
        
        const filePath = filePathInput.value.trim();
        if (!filePath) {
            showApiResponse({ error: 'File path is required' });
            return;
        }
        
        // Show loading state
        toggleLoading(true);
        
        try {
            // Encode the file path properly
            const encodedPath = encodeURIComponent(filePath);
            const fileUrl = `${API_BASE_URL}/${encodedPath}`;
            
            // Skip HEAD request and go directly to displaying the file
            // Determine content type based on file extension
            const contentType = guessContentType(filePath);
            
            // Display preview based on content type
            previewFile(fileUrl, contentType, filePath);
            
            // Show success response
            showApiResponse({
                message: 'File retrieval initiated',
                file_path: filePath,
                content_type: contentType,
                url: fileUrl
            });
        } catch (error) {
            console.error('Error fetching file:', error);
            showApiResponse({ error: error.message });
            showEmptyPreview('Error fetching file');
        } finally {
            toggleLoading(false);
        }
    }
    
    /**
     * Guess content type based on file extension
     * @param {string} filePath - Path to the file
     * @returns {string} - MIME type
     */
    function guessContentType(filePath) {
        const extension = filePath.split('.').pop().toLowerCase();
        const mimeTypes = {
            'pdf': 'application/pdf',
            'txt': 'text/plain',
            'html': 'text/html',
            'htm': 'text/html',
            'css': 'text/css',
            'js': 'text/javascript',
            'json': 'application/json',
            'xml': 'application/xml',
            'png': 'image/png',
            'jpg': 'image/jpeg',
            'jpeg': 'image/jpeg',
            'gif': 'image/gif',
            'svg': 'image/svg+xml',
            'mp4': 'video/mp4',
            'webm': 'video/webm',
            'mp3': 'audio/mpeg',
            'wav': 'audio/wav',
            'py': 'text/x-python',
            'md': 'text/markdown',
            'java': 'text/x-java',
            'c': 'text/x-c',
            'cpp': 'text/x-c++',
            'h': 'text/x-c',
            'cs': 'text/x-csharp'
        };
        
        return mimeTypes[extension] || 'application/octet-stream';
    }
    
    /**
     * Fetch with authentication headers
     * @param {string} url - URL to fetch
     * @param {object} options - Fetch options
     * @returns {Promise<Response>} - Fetch response
     */
    async function fetchWithAuth(url, options = {}) {
        // Add headers if not present
        options.headers = options.headers || {};
        
        // Add authorization header
        options.headers['Authorization'] = MOCK_AUTH_TOKEN;
        
        // Perform fetch with auth
        return fetch(url, options);
    }
    
    /**
     * Render the list of files
     * @param {Array} files - Array of file objects from the API
     */
    function renderFileList(files) {
        resetFileList();
        
        files.forEach(file => {
            const fileItem = document.createElement('div');
            fileItem.className = 'file-item';
            
            // Determine file icon based on MIME type
            const iconClass = getFileIconClass(file.mime_type);
            
            fileItem.innerHTML = `
                <div class="file-icon">
                    <i class="${iconClass}"></i>
                </div>
                <div class="file-details">
                    <div class="file-name">
                        ${file.file_name}
                        <span class="file-type ${getFileTypeClass(file.mime_type)}">
                            ${getFileTypeLabel(file.mime_type)}
                        </span>
                    </div>
                    <div class="file-meta">
                        ${formatFileSize(file.size_bytes)} | ${new Date(file.modified_time).toLocaleString()}
                    </div>
                </div>
            `;
            
            // Add click handler to fetch the file
            fileItem.addEventListener('click', () => {
                // Update the file path input with this file's path
                filePathInput.value = file.file_path;
                
                // Remove selected class from all items
                document.querySelectorAll('.file-item').forEach(item => {
                    item.classList.remove('selected');
                });
                
                // Add selected class to this item
                fileItem.classList.add('selected');
                
                // Automatically fetch and preview the file
                getFileForm.dispatchEvent(new Event('submit'));
            });
            
            fileList.appendChild(fileItem);
        });
    }
    
    /**
     * Show API response in the JSON display area
     * @param {Object} data - Response data to display
     */
    function showApiResponse(data) {
        jsonResponse.innerHTML = JSON.stringify(data, null, 2);
    }
    
    /**
     * Preview a file based on its content type
     * @param {string} fileUrl - URL to the file
     * @param {string} contentType - MIME type of the file
     * @param {string} fileName - Name of the file
     */
    function previewFile(fileUrl, contentType, fileName) {
        resetPreview();
        
        if (!contentType) {
            showEmptyPreview('Unknown file type');
            return;
        }
        
        // Add auth token to file URLs for preview elements that make separate requests
        const fileUrlWithToken = fileUrl + (fileUrl.includes('?') ? '&' : '?') + 'auth_token=' + encodeURIComponent(MOCK_AUTH_TOKEN.replace('Bearer ', ''));
        
        if (contentType.startsWith('image/')) {
            // Image preview
            previewContent.innerHTML = `
                <div class="preview-image">
                    <img src="${fileUrlWithToken}" alt="${fileName}" onerror="this.onerror=null; this.src=''; this.alt='Error loading image'; this.style.padding='20px'; this.style.color='red';">
                </div>
            `;
        } else if (contentType.startsWith('video/')) {
            // Video preview
            previewContent.innerHTML = `
                <div class="preview-video">
                    <video controls>
                        <source src="${fileUrlWithToken}" type="${contentType}">
                        Your browser does not support the video tag.
                    </video>
                </div>
            `;
        } else if (contentType.startsWith('audio/')) {
            // Audio preview
            previewContent.innerHTML = `
                <div class="preview-audio">
                    <audio controls>
                        <source src="${fileUrlWithToken}" type="${contentType}">
                        Your browser does not support the audio tag.
                    </audio>
                </div>
            `;
        } else if (contentType === 'application/pdf') {
            // PDF preview
            previewContent.innerHTML = `
                <div class="preview-pdf">
                    <iframe src="${fileUrlWithToken}" title="${fileName}"></iframe>
                </div>
            `;
        } else if (
            contentType.includes('text/') ||
            contentType.includes('application/json') ||
            contentType.includes('application/xml') ||
            /\.(js|py|java|c|cpp|h|html|css|md|txt|json|xml)$/.test(fileName)
        ) {
            // Text preview - fetch and display with auth header
            fetchWithAuth(fileUrl)
                .then(response => {
                    if (!response.ok) {
                        throw new Error(`HTTP error ${response.status}`);
                    }
                    return response.text();
                })
                .then(text => {
                    previewContent.innerHTML = `
                        <div class="preview-text">
                            <pre>${escapeHtml(text)}</pre>
                        </div>
                    `;
                })
                .catch(error => {
                    showEmptyPreview(`Error loading text: ${error.message}`);
                });
        } else {
            // Default preview - download link
            previewContent.innerHTML = `
                <div class="preview-unavailable">
                    <i class="fas fa-file-download"></i>
                    <p>Preview not available for this file type</p>
                    <a href="${fileUrlWithToken}" target="_blank" class="preview-download-btn">
                        <i class="fas fa-download"></i> Download File
                    </a>
                </div>
            `;
        }
    }
    
    // Helper Functions
    
    /**
     * Toggle the loading indicator
     * @param {boolean} isLoading - Whether to show or hide the loading indicator
     */
    function toggleLoading(isLoading) {
        loadingIndicator.style.display = isLoading ? 'flex' : 'none';
    }
    
    /**
     * Reset the file list to empty state
     */
    function resetFileList() {
        fileList.innerHTML = '';
    }
    
    /**
     * Show empty state for file list
     * @param {string} message - Optional message to display
     */
    function showEmptyFileList(message = 'No files found') {
        fileList.innerHTML = `
            <div class="initial-message">
                <i class="fas fa-folder-open"></i>
                <p>${message}</p>
            </div>
        `;
    }
    
    /**
     * Reset the preview content
     */
    function resetPreview() {
        previewContent.innerHTML = '';
    }
    
    /**
     * Show empty state for preview
     * @param {string} message - Optional message to display
     */
    function showEmptyPreview(message = 'Select a file to preview') {
        previewContent.innerHTML = `
            <div class="initial-message">
                <i class="fas fa-eye"></i>
                <p>${message}</p>
            </div>
        `;
    }
    
    /**
     * Format file size in human-readable format
     * @param {number} bytes - File size in bytes
     * @returns {string} Formatted file size
     */
    function formatFileSize(bytes) {
        if (bytes === 0) return '0 Bytes';
        
        const k = 1024;
        const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        
        return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
    }
    
    /**
     * Get Font Awesome icon class based on file MIME type
     * @param {string} mimeType - MIME type of the file
     * @returns {string} Font Awesome icon class
     */
    function getFileIconClass(mimeType) {
        if (!mimeType) return 'fas fa-file';
        
        if (mimeType.startsWith('image/')) return 'fas fa-file-image';
        if (mimeType.startsWith('video/')) return 'fas fa-file-video';
        if (mimeType.startsWith('audio/')) return 'fas fa-file-audio';
        if (mimeType === 'application/pdf') return 'fas fa-file-pdf';
        if (mimeType.includes('text/html')) return 'fas fa-file-code';
        if (mimeType.includes('text/css')) return 'fas fa-file-code';
        if (mimeType.includes('text/javascript')) return 'fas fa-file-code';
        if (mimeType.includes('application/json')) return 'fas fa-file-code';
        if (mimeType.includes('text/')) return 'fas fa-file-alt';
        if (mimeType.includes('application/zip')) return 'fas fa-file-archive';
        if (mimeType.includes('application/x-compressed')) return 'fas fa-file-archive';
        if (mimeType.includes('application/msword')) return 'fas fa-file-word';
        if (mimeType.includes('application/vnd.ms-excel')) return 'fas fa-file-excel';
        if (mimeType.includes('application/vnd.ms-powerpoint')) return 'fas fa-file-powerpoint';
        
        return 'fas fa-file';
    }
    
    /**
     * Get file type label based on MIME type
     * @param {string} mimeType - MIME type of the file
     * @returns {string} Human-readable file type
     */
    function getFileTypeLabel(mimeType) {
        if (!mimeType) return 'Unknown';
        
        if (mimeType.startsWith('image/')) return 'Image';
        if (mimeType.startsWith('video/')) return 'Video';
        if (mimeType.startsWith('audio/')) return 'Audio';
        if (mimeType === 'application/pdf') return 'PDF';
        if (mimeType.includes('text/html')) return 'HTML';
        if (mimeType.includes('text/css')) return 'CSS';
        if (mimeType.includes('text/javascript')) return 'JavaScript';
        if (mimeType.includes('application/json')) return 'JSON';
        if (mimeType.includes('text/')) return 'Text';
        if (mimeType.includes('application/zip')) return 'Archive';
        if (mimeType.includes('application/x-compressed')) return 'Archive';
        if (mimeType.includes('application/msword')) return 'Word';
        if (mimeType.includes('application/vnd.ms-excel')) return 'Excel';
        if (mimeType.includes('application/vnd.ms-powerpoint')) return 'PowerPoint';
        
        return 'Document';
    }
    
    /**
     * Get CSS class for file type badge
     * @param {string} mimeType - MIME type of the file
     * @returns {string} CSS class for the file type
     */
    function getFileTypeClass(mimeType) {
        if (!mimeType) return 'file-type-other';
        
        if (mimeType.startsWith('image/')) return 'file-type-image';
        if (mimeType.startsWith('video/')) return 'file-type-video';
        if (mimeType.startsWith('audio/')) return 'file-type-audio';
        if (mimeType.includes('text/') || 
            mimeType === 'application/pdf' || 
            mimeType.includes('application/msword')) return 'file-type-document';
        
        return 'file-type-other';
    }
    
    /**
     * Escape HTML to prevent XSS
     * @param {string} html - HTML string to escape
     * @returns {string} Escaped HTML string
     */
    function escapeHtml(html) {
        return html
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;')
            .replace(/'/g, '&#039;');
    }
}); 