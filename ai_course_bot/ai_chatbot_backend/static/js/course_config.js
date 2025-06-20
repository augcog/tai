// Course Configuration JavaScript
class CourseConfigApp {
    constructor() {
        this.baseURL = '/v1/course-admin';
        this.adminToken = null;
        this.currentEditingCourse = null;
        this.courses = [];
        
        this.initializeEventListeners();
        this.promptForAdminToken();
    }

    async promptForAdminToken() {
        // Check if admin token is stored in localStorage
        let token = localStorage.getItem('course_admin_token');
        
        if (token && token !== 'null') {
            this.adminToken = token;
            document.getElementById('admin-token-input').value = token;
            console.log('✅ Admin token loaded from storage');
            this.loadCourses();
        } else {
            this.showResponse({ 
                error: 'Admin token required. Please enter your admin token above.',
                instructions: 'Contact the development team to obtain the secure admin token for course management.'
            }, true);
        }
    }

    setAdminToken() {
        const token = document.getElementById('admin-token-input').value.trim();
        
        if (!token) {
            this.showResponse({ 
                error: 'Please enter an admin token' 
            }, true);
            return;
        }
        
        this.adminToken = token;
        localStorage.setItem('course_admin_token', token);
        console.log('✅ Admin token configured');
        
        this.showResponse({ 
            message: 'Admin token set successfully. You can now manage courses.',
            token_preview: token.substring(0, 10) + '...'
        });
        
        // Test the token by loading courses
        this.loadCourses();
    }

    initializeEventListeners() {
        // Form submissions
        document.getElementById('create-course-form').addEventListener('submit', (e) => {
            e.preventDefault();
            this.createCourse();
        });

        document.getElementById('edit-course-form').addEventListener('submit', (e) => {
            e.preventDefault();
            this.updateCourse();
        });

        // Button clicks
        document.getElementById('set-admin-token-btn').addEventListener('click', () => {
            this.setAdminToken();
        });

        document.getElementById('refresh-courses-btn').addEventListener('click', () => {
            this.loadCourses();
        });

        document.getElementById('clear-response-btn').addEventListener('click', () => {
            this.clearResponse();
        });

        document.getElementById('cancel-edit-btn').addEventListener('click', () => {
            this.cancelEdit();
        });

        // Modal events
        document.getElementById('confirm-yes').addEventListener('click', () => {
            this.executeConfirmedAction();
        });

        document.getElementById('confirm-no').addEventListener('click', () => {
            this.hideModal();
        });

        // Access type change handler for both forms - with debugging
        const accessTypeInput = document.getElementById('access-type-input');
        const editAccessType = document.getElementById('edit-access-type');
        
        if (accessTypeInput) {
            accessTypeInput.addEventListener('change', (e) => {
                console.log('Access type changed to:', e.target.value);
                this.toggleSchoolField('school-input-group', e.target.value);
            });
            
            // Add click event for debugging
            accessTypeInput.addEventListener('click', (e) => {
                console.log('Access type dropdown clicked');
            });
        } else {
            console.error('Access type input not found');
        }

        if (editAccessType) {
            editAccessType.addEventListener('change', (e) => {
                console.log('Edit access type changed to:', e.target.value);
                this.toggleSchoolField('edit-school-input-group', e.target.value);
            });
        }

        // Allow Enter key to set admin token
        document.getElementById('admin-token-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                this.setAdminToken();
            }
        });
    }

    toggleSchoolField(groupId, accessType) {
        const schoolGroup = document.getElementById(groupId);
        if (!schoolGroup) {
            console.warn(`School field group ${groupId} not found`);
            return;
        }
        
        // Since we only have 'public' and 'private' now, always hide the school field
        schoolGroup.style.display = 'none';
    }

    showLoading() {
        document.getElementById('loading-indicator').style.display = 'flex';
    }

    hideLoading() {
        document.getElementById('loading-indicator').style.display = 'none';
    }

    showResponse(data, isError = false) {
        const responseElement = document.getElementById('json-response');
        responseElement.textContent = JSON.stringify(data, null, 2);
        responseElement.className = `json-response ${isError ? 'error' : 'success'}`;
    }

    clearResponse() {
        const responseElement = document.getElementById('json-response');
        responseElement.textContent = '';
        responseElement.className = 'json-response';
    }

    showModal(message, action) {
        document.getElementById('confirmation-message').textContent = message;
        document.getElementById('confirmation-modal').style.display = 'flex';
        this.pendingAction = action;
    }

    hideModal() {
        document.getElementById('confirmation-modal').style.display = 'none';
        this.pendingAction = null;
    }

    executeConfirmedAction() {
        if (this.pendingAction) {
            this.pendingAction();
        }
        this.hideModal();
    }

    async apiRequest(url, options = {}) {
        try {
            this.showLoading();
            
            // Prepare headers
            const headers = {
                'Content-Type': 'application/json',
                ...options.headers
            };
            
            // Only add Authorization header for write operations
            const method = options.method || 'GET';
            const writeOperations = ['POST', 'PUT', 'PATCH', 'DELETE'];
            
            if (writeOperations.includes(method.toUpperCase())) {
                if (!this.adminToken) {
                    throw new Error('Admin token is required for this operation. Please refresh and enter your token.');
                }
                headers['Authorization'] = `Bearer ${this.adminToken}`;
            }
            
            const response = await fetch(url, {
                headers,
                ...options
            });

            let data;
            try {
                data = await response.json();
            } catch (parseError) {
                // If response isn't JSON, create an error object
                data = { 
                    detail: `Invalid response format: ${response.statusText}`,
                    status: response.status 
                };
            }
            
            if (!response.ok) {
                // Handle specific auth errors
                if (response.status === 403 || response.status === 401) {
                    // Clear stored token on auth failure
                    localStorage.removeItem('course_admin_token');
                    this.adminToken = null;
                    
                    throw new Error(
                        data.detail || 
                        'Authentication failed. The admin token may be incorrect. Please refresh and try again.'
                    );
                }
                
                throw new Error(data.detail || data.message || `HTTP ${response.status}: ${response.statusText}`);
            }

            return data;
        } catch (error) {
            console.error('API Error:', error);
            
            // Better error handling - serialize any type of error properly
            let errorMessage;
            if (error instanceof TypeError && error.message.includes('fetch')) {
                errorMessage = 'Network error: Unable to connect to server';
            } else if (error.message) {
                errorMessage = error.message;
            } else if (typeof error === 'string') {
                errorMessage = error;
            } else {
                errorMessage = 'Unknown error occurred';
            }
            
            this.showResponse({ 
                error: errorMessage,
                timestamp: new Date().toISOString(),
                url: url,
                method: options.method || 'GET'
            }, true);
            
            throw error;
        } finally {
            this.hideLoading();
        }
    }

    async loadCourses() {
        try {
            const data = await this.apiRequest(`${this.baseURL}/`);
            this.courses = data.courses || [];
            this.renderCourses();
            this.showResponse(data);
        } catch (error) {
            console.error('Load courses error:', error);
            this.renderError('Failed to load courses. Please check if the server is running.');
            // Don't show the error in the response panel again since apiRequest already did
        }
    }

    async createCourse() {
        const formData = this.getFormData('create-course-form');
        
        // Basic validation
        if (!formData.course_name || !formData.server_url) {
            this.showResponse({ 
                error: 'Please fill in all required fields (Course Name, Server URL)' 
            }, true);
            return;
        }
        
        try {
            const newCourse = await this.apiRequest(`${this.baseURL}/`, {
                method: 'POST',
                body: JSON.stringify(formData)
            });

            this.showResponse(newCourse);
            this.resetForm('create-course-form');
            this.loadCourses(); // Refresh the list
        } catch (error) {
            console.error('Create course error:', error);
            // Error already shown by apiRequest
        }
    }

    async updateCourse() {
        if (!this.currentEditingCourse) {
            this.showResponse({ error: 'No course selected for editing' }, true);
            return;
        }

        const formData = this.getEditFormData();
        
        // Basic validation
        if (!formData.course_name || !formData.server_url) {
            this.showResponse({ 
                error: 'Please fill in all required fields (Course Name, Server URL)' 
            }, true);
            return;
        }
        
        try {
            const updatedCourse = await this.apiRequest(`${this.baseURL}/${this.currentEditingCourse.id}`, {
                method: 'PUT',
                body: JSON.stringify(formData)
            });

            this.showResponse(updatedCourse);
            this.cancelEdit();
            this.loadCourses(); // Refresh the list
        } catch (error) {
            console.error('Update course error:', error);
            // Error already shown by apiRequest
        }
    }

    async toggleCourse(courseId, courseName) {
        this.showModal(
            `Are you sure you want to toggle the status of "${courseName}"?`,
            async () => {
                try {
                    const updatedCourse = await this.apiRequest(`${this.baseURL}/${courseId}/toggle`, {
                        method: 'PATCH'
                    });

                    this.showResponse(updatedCourse);
                    this.loadCourses(); // Refresh the list
                } catch (error) {
                    console.error('Toggle course error:', error);
                }
            }
        );
    }

    async deleteCourse(courseId, courseName) {
        this.showModal(
            `Are you sure you want to delete "${courseName}"? This action cannot be undone.`,
            async () => {
                try {
                    await this.apiRequest(`${this.baseURL}/${courseId}`, {
                        method: 'DELETE'
                    });

                    this.showResponse({ message: 'Course deleted successfully' });
                    this.loadCourses(); // Refresh the list
                } catch (error) {
                    console.error('Delete course error:', error);
                }
            }
        );
    }

    editCourse(course) {
        this.currentEditingCourse = course;
        this.populateEditForm(course);
        this.showEditPanel();
    }

    populateEditForm(course) {
        document.getElementById('edit-course-name').value = course.course_name;
        document.getElementById('edit-server-url').value = course.server_url;
        document.getElementById('edit-access-type').value = course.access_type;
        document.getElementById('edit-school').value = course.school || '';
        document.getElementById('edit-enabled').checked = course.enabled;

        // Toggle school field visibility
        this.toggleSchoolField('edit-school-input-group', course.access_type);
    }

    showEditPanel() {
        document.getElementById('edit-panel').style.display = 'block';
    }

    cancelEdit() {
        this.currentEditingCourse = null;
        document.getElementById('edit-panel').style.display = 'none';
        this.resetForm('edit-course-form');
    }

    getFormData(formId) {
        const form = document.getElementById(formId);
        const formData = new FormData(form);
        
        return {
            course_name: document.getElementById('course-name-input').value,
            server_url: document.getElementById('server-url-input').value,
            access_type: document.getElementById('access-type-input').value,
            school: document.getElementById('school-input').value || null,
            enabled: document.getElementById('enabled-input').checked
        };
    }

    getEditFormData() {
        return {
            course_name: document.getElementById('edit-course-name').value,
            server_url: document.getElementById('edit-server-url').value,
            access_type: document.getElementById('edit-access-type').value,
            school: document.getElementById('edit-school').value || null,
            enabled: document.getElementById('edit-enabled').checked
        };
    }

    resetForm(formId) {
        const form = document.getElementById(formId);
        form.reset();
        
        // Reset specific fields if needed
        if (formId === 'create-course-form') {
            document.getElementById('enabled-input').checked = true;
            document.getElementById('school-input-group').style.display = 'none';
        }
    }

    renderCourses() {
        const container = document.getElementById('course-list');
        
        if (this.courses.length === 0) {
            container.innerHTML = `
                <div class="initial-message">
                    <i class="fas fa-graduation-cap"></i>
                    <p>No courses found. Create your first course above.</p>
                </div>
            `;
            return;
        }

        const tableHTML = `
            <table class="course-table">
                <thead>
                    <tr>
                        <th>Name</th>
                        <th>Course ID</th>
                        <th>Server URL</th>
                        <th>Status</th>
                        <th>Access Type</th>
                        <th>School</th>
                        <th>Actions</th>
                    </tr>
                </thead>
                <tbody>
                    ${this.courses.map(course => this.renderCourseRow(course)).join('')}
                </tbody>
            </table>
        `;

        container.innerHTML = tableHTML;
    }

    renderCourseRow(course) {
        const statusClass = course.enabled ? 'enabled' : 'disabled';
        const statusText = course.enabled ? 'Enabled' : 'Disabled';
        
        return `
            <tr>
                <td>
                    <div class="course-name">${this.escapeHtml(course.course_name)}</div>
                </td>
                <td>
                    <div class="course-id">${course.course_id}</div>
                </td>
                <td>
                    <a href="${this.escapeHtml(course.server_url)}" target="_blank" rel="noopener">
                        ${this.escapeHtml(course.server_url)}
                    </a>
                </td>
                <td>
                    <span class="course-status ${statusClass}">${statusText}</span>
                </td>
                <td>
                    <span class="access-type ${course.access_type}">${course.access_type}</span>
                </td>
                <td>${course.school ? this.escapeHtml(course.school) : '-'}</td>
                <td>
                    <div class="course-actions">
                        <button class="action-btn toggle-btn" onclick="app.toggleCourse(${course.id}, '${this.escapeHtml(course.course_name)}')" title="Toggle Status">
                            <i class="fas fa-power-off"></i>
                        </button>
                        <button class="action-btn edit-btn" onclick="app.editCourse(${JSON.stringify(course).replace(/"/g, '&quot;')})" title="Edit">
                            <i class="fas fa-edit"></i>
                        </button>
                        <button class="action-btn delete-btn" onclick="app.deleteCourse(${course.id}, '${this.escapeHtml(course.course_name)}')" title="Delete">
                            <i class="fas fa-trash"></i>
                        </button>
                    </div>
                </td>
            </tr>
        `;
    }

    renderError(message) {
        const container = document.getElementById('course-list');
        container.innerHTML = `
            <div class="initial-message">
                <i class="fas fa-exclamation-triangle" style="color: var(--danger-color);"></i>
                <p>${message}</p>
            </div>
        `;
    }

    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
}

// Initialize the app when the DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.app = new CourseConfigApp();
}); 