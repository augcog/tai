import { auth } from '@/tai/utils/auth'
import { LoginButton } from '@/tai/components/login-button'
import { redirect } from 'next/navigation'
import { Chat } from '@/tai/components/chat'
import { nanoid } from '@/tai/lib/utils'

export default async function SignInPage() {
  const session = await auth()
  const id = nanoid()

  // redirect to home if user is already logged in
  if (session?.user) {
    redirect('/')
  }

  return (
    // <p>hi</p>
    <Chat id={id} />
    // <LoginButton />
  )
}
