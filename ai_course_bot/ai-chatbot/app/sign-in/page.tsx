import { auth } from '@/auth'
import { LoginButton } from '@/components/login-button'
import { redirect } from 'next/navigation'
import { Chat } from '@/components/chat'
import { nanoid } from '@/lib/utils'

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
