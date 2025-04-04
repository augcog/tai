# Admin Dashboard

## Overview

The Admin Dashboard provides a web interface for managing the application's local database. It currently supports course record management but is designed to be extensible for future database-related features.

## Accessing the Dashboard

The admin dashboard is accessible at the `/admin` endpoint of your application. For example, if your application is running at `http://localhost:8000`, you can access the admin dashboard at:

```
http://localhost:8000/admin
```

## Features

### Current Functionality

- **Course Management**: Create, read, update, and delete course records
  - View all courses in a sortable and searchable table
  - Filter courses by name or course code
  - Detailed view of individual course information
  
### Course Fields

- **Course Name**: The full name of the course
- **Course Code**: Unique identifier for the course (e.g., CS61A)
- **IP Address**: The IP address associated with the course
- **Access Type**: Controls visibility and access requirements
  - `public`: Available to all users
  - `login_required`: Requires school login credentials
  - `private`: Restricted access
- **School**: School name (required only when Access Type is set to 'login_required')

## Database Information

The application uses SQLite as the default database engine for simplicity. The database file is located at:

```
./courses.db
```

in the root directory of the application.

## Extensibility

The Admin Dashboard is built using SQLAdmin with FastAPI, making it highly extensible:

1. **Adding New Models**: New database models can be easily added to the dashboard by:
   - Creating a new SQLAlchemy model in the appropriate location in the codebase
   - Creating a corresponding ModelView class in the admin.py file
   - Registering the new view with the admin instance

2. **Customizing Views**: Each ModelView can be customized with:
   - Custom column displays
   - Sorting and filtering options
   - Form layouts and validators
   - Custom icons and naming

3. **Future Capabilities**: The dashboard architecture supports:
   - User authentication and authorization
   - Custom actions and workflows
   - Dashboard analytics and reporting
   - Integration with external services

## Development

To extend the dashboard with a new model:

1. Create your model in `app/core/models/`
2. Create a new ModelView class in `app/admin/admin.py`
3. Register your view in the `setup_admin` function
4. Update the database schema as needed

## Testing

Tests for the admin dashboard can be found in:

```
/tests/unit_tests/test_admin_panel/
``` 