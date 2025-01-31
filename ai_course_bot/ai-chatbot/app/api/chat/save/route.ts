import { nanoid } from '@/tai/lib/utils'
import { auth } from '@/tai/utils/auth'
import { saveChat } from '@/tai/utils/actions'

export async function POST(req: Request) {
  const json = await req.json()
  const userId: string = (await auth())?.user.id ?? nanoid()
  const title = json.messages[0].content.substring(0, 100)
  await saveChat(title, json.messages, userId, json.id)
  return new Response()
}
