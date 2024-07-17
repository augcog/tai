import { nanoid } from '@/app/lib/utils'
import { Chat } from '@/app/components/chat'

export default function IndexPage() {
  const id = nanoid()

  return <Chat id={id} />
}
