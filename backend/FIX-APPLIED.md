# Fix Applied: function_tool Name Parameter

## Issue
```
TypeError: function_tool() got an unexpected keyword argument 'name'
```

## Root Cause
The `function_tool()` decorator from OpenAI Agents SDK doesn't accept a `name` parameter. It automatically uses the function name.

## Fix Applied
Changed from:
```python
async def _search_textbook_impl(...):
    ...

search_textbook = function_tool(_search_textbook_impl, name="search_textbook")
```

To:
```python
@function_tool
async def search_textbook(...):
    ...
```

## Result
✅ Tools now use the decorator directly  
✅ Function names are automatically used as tool names  
✅ Backend should start without errors

## Test
```bash
cd backend
uv run uvicorn rag_chatbot_backend.api.main:app --reload
```

Should start successfully now!

