import { nanoid } from '@/tai/lib/utils'
import { Chat } from '@/tai/components/chat'

export default function IndexPage() {
  const id = nanoid()

  return <Chat id={id} />
}
