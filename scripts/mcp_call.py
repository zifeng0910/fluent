"""Call the locally bound PyFluent MCP over its real HTTP transport."""
import asyncio, json, sys
from datetime import datetime, timezone
from pathlib import Path
from fastmcp import Client

async def main():
    args=json.loads(Path(sys.argv[2]).read_text(encoding='utf-8-sig')) if len(sys.argv)>2 else {}
    async with Client('http://127.0.0.1:8765/mcp', timeout=300) as client:
        if sys.argv[1]=='list_tools':
            result=[t.model_dump(mode='json') for t in await client.list_tools()]
        else:
            reply=await client.call_tool(sys.argv[1],args,raise_on_error=False)
            result={'data':reply.data,'is_error':reply.is_error,'content':[c.model_dump(mode='json') for c in reply.content]}
        record={'time':datetime.now(timezone.utc).isoformat(),'tool':sys.argv[1],'arguments':args,'result':result}
        out=Path('H:/fluent/logs');out.mkdir(exist_ok=True)
        (out/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+sys.argv[1]+'.json')).write_text(json.dumps(record,indent=2,default=str),encoding='utf-8')
        print(json.dumps(result,indent=2,default=str))

asyncio.run(main())
