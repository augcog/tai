# 📋 Postman Collections

This directory contains comprehensive Postman collections for testing and exploring the API endpoints. Perfect for frontend team collaboration and API exploration.

## 📁 Available Collections

### `files_api_collection.json` - Modern Files API
**Complete collection for the modern Files API with UUID-based access**

#### 🎯 **What's Included:**
- **📁 File Listing & Discovery** - Auto-discovery with various filters
- **🔍 Search & Advanced Filtering** - Powerful search and combined filters  
- **📄 File Operations** - Metadata retrieval and secure downloads
- **📊 Pagination & Limits** - Efficient handling of large collections
- **📈 Statistics & System Info** - System health and monitoring
- **🎯 Real Examples** - Using actual files from the data directory

#### 🚀 **Quick Start:**
1. Import `files_api_collection.json` into Postman
2. Set environment variables:
   - `baseUrl`: `http://localhost:8000`
   - `authToken`: Your auth token (if authentication enabled)
   - `sampleFileUuid`: A valid file UUID for testing
3. Start with "📁 File Listing & Discovery" → "List All Files (Basic)"

## 🔧 **Environment Variables**

Set these variables in your Postman environment:

```json
{
  "baseUrl": "http://localhost:8000",
  "authToken": "YOUR_AUTH_TOKEN",
  "sampleFileUuid": "550e8400-e29b-41d4-a716-446655440000"
}
```

## 📖 **Collection Organization**

### 📁 **File Listing & Discovery**
- **List All Files** - Basic listing with auto-discovery
- **Filter by Course** - CS61A, CS61B specific files
- **Filter by Category** - Documents, videos, audio, other

### 🔍 **Search & Advanced Filtering**
- **Search by Name** - Find files by filename/title
- **Combined Filters** - Course + Category combinations
- **Triple Filters** - Course + Category + Search

### 📄 **File Operations**
- **Get Metadata** - Detailed file information by UUID
- **Download File** - Secure file download with proper headers

### 📊 **Pagination & Limits**
- **Page 1 & 2** - Pagination examples for large collections
- **Custom Limits** - Different page sizes (10, 50, 100 items)

### 📈 **Statistics & System Info**
- **File Statistics** - Total counts, course breakdown, system info

### 🎯 **Real Examples**
- **CS61A Lab Material** - Actual lab files
- **Python Code Files** - Find .py files across courses
- **PDF Documents** - Study materials and exams

## 💡 **Usage Tips**

### **For Frontend Developers:**
1. **Start with Basic Listing** to understand the response format
2. **Try Different Filters** to see how to build filtered views
3. **Test Pagination** for implementing infinite scroll or pagination
4. **Use Real Examples** to work with actual data structure

### **For API Testing:**
1. **Check Response Schemas** - All responses include example data
2. **Test Error Cases** - Try invalid UUIDs, missing files
3. **Verify Filters** - Ensure filtering works as expected
4. **Monitor Performance** - Check response times with large datasets

### **For Integration:**
1. **UUID-based Access** - All file operations use secure UUIDs
2. **Auto-discovery** - New files appear automatically
3. **Clean Filtering** - Simple, predictable filter parameters
4. **Proper Headers** - Download endpoints return correct MIME types

## 🔄 **API Endpoints Summary**

```
GET /v1/files                        # List files with auto-discovery
GET /v1/files?course_code=CS61A      # Filter by course
GET /v1/files?category=document      # Filter by category  
GET /v1/files?search=lab             # Search files
GET /v1/files/{uuid}                 # Get file metadata
GET /v1/files/{uuid}/download        # Download file securely
GET /v1/files/stats/summary          # System statistics
```

## 📝 **Response Format**

All listing endpoints return:
```json
{
  "files": [...],           // Array of file objects
  "total_count": 42,        // Total matching files
  "page": 1,                // Current page
  "limit": 100,             // Items per page
  "has_next": false,        // More pages available
  "has_prev": false,        // Previous pages available
  "filters_applied": {...}  // Applied filters summary
}
```

## 🎉 **Benefits for Frontend Team**

✅ **Complete Examples** - Every endpoint with realistic data  
✅ **Diverse Scenarios** - Different filter combinations and use cases  
✅ **Real Data** - Examples using actual files from data directory  
✅ **Clear Organization** - Logical grouping by functionality  
✅ **Response Samples** - See exactly what to expect  
✅ **Easy Testing** - Import and start testing immediately  

Perfect for building file browsers, search interfaces, and media galleries! 🚀
